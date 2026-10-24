"""Load compiled models in Titanfall 2 through Northstar and screenshot them.

Builds an isolated Northstar profile (default C:/Games/Titanfall2/R2ModelTest) with
the core Northstar mods plus a generated test mod that precaches each model, spawns
it as a prop_dynamic in front of the local player, takes a screenshot and quits.

usage: python tools/ingame_test.py [--game DIR] [--map MAP] [--timeout S] model.mdl=models/path.mdl ...
Each argument is a local compiled file and the game path it should be mounted at.
"""
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

SERVER_SCRIPT = r'''
global function R2C_ModelTest_Init

struct { array<asset> models } file

void function R2C_ModelTest_Init()
{
    // local test server: accept our own client without master-server auth
    SetConVarBool( "ns_auth_allow_insecure", true )
    printl( "[R2C] ns_auth_allow_insecure = " + GetConVarBool( "ns_auth_allow_insecure" ) )
%PRECACHE%
    AddCallback_OnPlayerRespawned( R2C_OnSpawn )
}

void function R2C_OnSpawn( entity player )
{
    thread R2C_SpawnThread( player )
}

void function R2C_SpawnThread( entity player )
{
    wait 1.0
    if ( !IsValid( player ) )
        return
    player.SetInvulnerable()
    vector fwd = AnglesToForward( < 0, player.EyeAngles().y, 0 > )
    vector right = AnglesToRight( < 0, player.EyeAngles().y, 0 > )
    int i = 0
    foreach ( asset m in file.models )
    {
        vector pos = player.GetOrigin() + fwd * %DIST% + right * ( ( i - ( file.models.len() - 1 ) * 0.5 ) * %SPACING% + %LATERAL% )
%SPAWN%
        if ( IsValid( prop ) )
        {
            printl( "[R2C] spawned " + m + " at " + pos + " mins " + prop.GetBoundingMins() + " maxs " + prop.GetBoundingMaxs() )
            // collision check: trace down onto the prop from above
            TraceResults tr = TraceLine( pos + < 0, 0, 400 >, pos - < 0, 0, 16 >, [ player ], TRACE_MASK_SHOT, TRACE_COLLISION_GROUP_NONE )
            bool hitProp = IsValid( tr.hitEnt ) && tr.hitEnt == prop
            printl( "[R2C] trace " + m + " hitProp=" + hitProp + " endZ=" + ( tr.endPos.z - pos.z ) )
            int seq = prop.LookupSequence( "%SEQ%" )
            if ( seq >= 0 && !%PHYSICS% )
            {
                prop.Anim_Play( "%SEQ%" )
                printl( "[R2C] animation " + m + " sequence=%SEQ% index=" + seq )
            }
            else if ( !%PHYSICS% )
                printl( "[R2C] MISSING sequence %SEQ% on " + m )
%PROBE%
%RAGDOLL%
%PHYSICS_PROBE%
        }
        else
            printl( "[R2C] FAILED to spawn " + m )
        i++
    }
    printl( "[R2C] spawn done" )
}

void function R2C_PhysicsProbe( entity prop )
{
    vector first = prop.GetOrigin()
    wait 1.0
    if ( !IsValid( prop ) ) return
    vector second = prop.GetOrigin()
    printl( "[R2C] physics drop=" + ( first.z - second.z ) + " from=" + first + " to=" + second )
}

void function R2C_Probe( entity prop )
{
    wait 0.1
    if ( !IsValid( prop ) ) return
    int attachment = prop.LookupAttachment( "%ATTACHMENT%" )
    if ( attachment <= 0 )
    {
        printl( "[R2C] MISSING probe attachment %ATTACHMENT%" )
        return
    }
    vector first = prop.GetAttachmentOrigin( attachment )
    wait 0.4
    if ( !IsValid( prop ) ) return
    vector second = prop.GetAttachmentOrigin( attachment )
    printl( "[R2C] attachment motion=" + Distance( first, second ) + " from=" + first + " to=" + second )
}

void function R2C_Ragdoll( entity prop )
{
    wait 2.0
    if ( !IsValid( prop ) ) return
    prop.Anim_Stop()
    prop.SetContinueAnimatingAfterRagdoll( false )
    prop.SetOrigin( prop.GetOrigin() + < 0, 0, 80 > )
    prop.BecomeRagdoll( < 0, 0, 180 >, true )
    printl( "[R2C] ragdoll requested" )
    wait 1.0
    if ( IsValid( prop ) )
        printl( "[R2C] ragdoll entity origin=" + prop.GetOrigin() )
}
'''

CLIENT_SCRIPT = r'''
global function R2C_ModelTest_ClientInit

void function R2C_ModelTest_ClientInit()
{
    thread R2C_Capture()
}

void function R2C_Capture()
{
    wait %SHOTDELAY%
    entity player = GetLocalClientPlayer()
    if ( !IsValid( player ) )
    {
        printl( "[R2C] no local player for screenshot" )
        return
    }
    player.ClientCommand( "cl_drawhud 0" )
    wait 1.0
    player.ClientCommand( "screenshot" )
    printl( "[R2C] screenshot requested" )
    wait 3.0
    if ( %QUIT% )
        player.ClientCommand( "quit" )
}
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--game", default=r"C:\Games\Titanfall2")
    ap.add_argument("--profile", default="R2ModelTest")
    ap.add_argument("--map", default="mp_lf_stacks")
    ap.add_argument("--mode", default="tdm")
    ap.add_argument("--timeout", type=float, default=240)
    ap.add_argument("--dist", type=float, default=160)
    ap.add_argument("--spacing", type=float, default=120)
    ap.add_argument("--lateral", type=float, default=0, help="offset the model group sideways for screenshots")
    ap.add_argument("--sequence", default="idle")
    ap.add_argument("--probe-attachment", default="", help="log attachment motion during the sequence")
    ap.add_argument("--ragdoll", action="store_true", help="convert spawned models to ragdolls after two seconds")
    ap.add_argument("--npc", action="store_true", help="apply models to soldier NPCs for death/ragdoll testing")
    ap.add_argument("--physics", action="store_true", help="spawn physical props above the floor and log their fall")
    ap.add_argument("--shot-delay", type=float, default=12)
    ap.add_argument("--keep-open", action="store_true")
    ap.add_argument("--report", help="write a JSON validation report")
    ap.add_argument("models", nargs="+")
    a = ap.parse_args()
    if a.npc and a.physics:
        ap.error('--npc and --physics are mutually exclusive')

    prof = os.path.join(a.game, a.profile)
    mods = os.path.join(prof, "mods")
    os.makedirs(mods, exist_ok=True)
    for core in ("Northstar.Client", "Northstar.Custom", "Northstar.CustomServers"):
        dst = os.path.join(mods, core)
        if not os.path.isdir(dst):
            shutil.copytree(os.path.join(a.game, "R2Northstar", "mods", core), dst)
    mod = os.path.join(mods, "R2C.ModelTest")
    profile_root = os.path.normcase(os.path.realpath(prof))
    generated_root = os.path.normcase(os.path.realpath(mod))
    if os.path.commonpath([profile_root, generated_root]) != profile_root:
        raise ValueError('generated test mod resolves outside the selected profile')
    if os.path.isdir(mod):
        shutil.rmtree(mod)
    os.makedirs(os.path.join(mod, "mod", "scripts", "vscripts"))
    precache = []
    for spec in a.models:
        if "=" in spec:
            local, gamepath = spec.split("=", 1)
        else:
            local, gamepath = spec, "models/r2c_test/" + os.path.basename(spec)
        dst = os.path.join(mod, "mod", gamepath)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(local, dst)
        precache.append('    file.models.append( $"%s" )\n    PrecacheModel( $"%s" )' % (gamepath, gamepath))
    sv = SERVER_SCRIPT.replace("%PRECACHE%", "\n".join(precache)).replace("%DIST%", str(a.dist)) \
        .replace("%SPACING%", str(a.spacing)).replace("%SEQ%", a.sequence)
    sv = sv.replace("%LATERAL%", str(a.lateral))
    sv = sv.replace("%PROBE%", '            thread R2C_Probe( prop )' if a.probe_attachment else '') \
        .replace("%RAGDOLL%", '            thread R2C_Ragdoll( prop )' if a.ragdoll else '') \
        .replace("%ATTACHMENT%", a.probe_attachment)
    spawn = '        entity prop = CreatePropDynamic( m, pos, < 0, player.EyeAngles().y + 180, 0 >, SOLID_VPHYSICS )'
    if a.npc:
        spawn = '''        entity prop = CreateNPC( "npc_soldier", player.GetTeam(), pos, < 0, player.EyeAngles().y + 180, 0 > )
        DispatchSpawn( prop )
        prop.SetModel( m )'''
    elif a.physics:
        spawn = '        entity prop = CreatePropPhysics( m, pos + < 0, 0, 100 >, < 0, player.EyeAngles().y + 180, 0 > )'
    sv = sv.replace("%SPAWN%", spawn).replace("%NPC%", 'true' if a.npc else 'false') \
        .replace("%PHYSICS%", 'true' if a.physics else 'false')
    sv = sv.replace("%PHYSICS_PROBE%", '            thread R2C_PhysicsProbe( prop )' if a.physics else '')
    cl = CLIENT_SCRIPT.replace("%SHOTDELAY%", str(a.shot_delay)).replace("%QUIT%", "false" if a.keep_open else "true")
    open(os.path.join(mod, "mod", "scripts", "vscripts", "r2c_modeltest.nut"), "w").write(sv)
    open(os.path.join(mod, "mod", "scripts", "vscripts", "r2c_modeltest_cl.nut"), "w").write(cl)
    open(os.path.join(mod, "mod.json"), "w").write('''{
  "Name": "R2C.ModelTest",
  "Version": "0.1.0",
  "LoadPriority": 100,
  "Description": "r2-model-compiler in-game model test",
  "Scripts": [
    {"Path": "r2c_modeltest.nut", "RunOn": "SERVER && MP", "ServerCallback": {"After": "R2C_ModelTest_Init"}},
    {"Path": "r2c_modeltest_cl.nut", "RunOn": "CLIENT && MP", "ClientCallback": {"After": "R2C_ModelTest_ClientInit"}}
  ]
}
''')
    open(os.path.join(prof, "enabledmods.json"), "w").write('''{
    "Northstar.Client": true, "Northstar.Custom": true, "Northstar.CustomServers": true, "R2C.ModelTest": true
}''')
    shots = os.path.join(a.game, "r2", "screenshots")
    before = set(glob.glob(os.path.join(shots, "*")))
    logs_dir = os.path.join(prof, "logs")
    logs_before = set(glob.glob(os.path.join(logs_dir, "*.txt")))
    # Launch flags follow the apex-to-titanfall-2 map harness: a LAN listen server that
    # never reports to the Northstar master server.
    args = [os.path.join(a.game, "NorthstarLauncher.exe"), "-profile=" + a.profile, "-multiple", "-novid", "-nosound",
            "-windowed", "-w", "1280", "-h", "720", "+ip", "127.0.0.1", "+hostport", "37042", "+sv_lan", "1",
            "+ns_report_server_to_masterserver", "0", "+ns_report_sp_server_to_masterserver", "0", "+sv_cheats", "1", "+ns_auth_allow_insecure", "1",
            "+setplaylist", "private_match", "+mp_gamemode", a.mode, "+map", a.map]
    print("launching:", " ".join(args))
    p = subprocess.Popen(args, cwd=a.game)
    t0 = time.time()
    timed_out = False
    while time.time() - t0 < a.timeout:
        if p.poll() is not None:
            break
        time.sleep(2)
    if p.poll() is None and not a.keep_open:
        timed_out = True
        print("timeout; stopping this test instance")
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    new_shots = sorted(set(glob.glob(os.path.join(shots, "*"))) - before)
    new_logs = sorted(set(glob.glob(os.path.join(logs_dir, "*.txt"))) - logs_before)
    print("screenshots:", new_shots)
    log_text = ''
    for lg in new_logs:
        print("log:", lg)
        with open(lg, errors="replace") as log:
            contents = log.read()
        log_text += contents
        for line in contents.splitlines():
            l = line.lower()
            if "[r2c]" in l or "error" in l or "mdl" in l or "model" in l and ("fail" in l or "bad" in l):
                print("   ", line.rstrip())
    failures = []
    if timed_out: failures.append('test timed out')
    for marker in ('SCRIPT ERROR:', 'COMPILE ERROR', '[R2C] FAILED', '[R2C] MISSING'):
        if marker in log_text: failures.append(marker)
    spawns = log_text.count('[R2C] spawned ')
    if spawns != len(a.models): failures.append('not all models spawned')
    if not new_shots and not a.keep_open: failures.append('no screenshot captured')
    drops = [float(value) for value in re.findall(r'\[R2C\] physics drop=([-+\d.eE]+)', log_text)]
    if a.physics and (len(drops) != len(a.models) or any(drop <= 0 for drop in drops)):
        failures.append('physics movement was not demonstrated')
    report = {'success': not failures, 'failures': failures, 'spawns': spawns,
              'physics_drop': drops, 'attachment_motion': [float(value) for value in
              re.findall(r'\[R2C\] attachment motion=([-+\d.eE]+)', log_text)],
              'logs': new_logs, 'screenshots': new_shots, 'models': a.models}
    if a.report:
        with open(a.report, 'w') as output: json.dump(report, output, indent=2)
    print('validation:', 'FAILED: ' + '; '.join(failures) if failures else 'passed')
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
