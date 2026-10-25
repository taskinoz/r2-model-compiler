// Growable little-endian byte buffer with deferred fixups for relative offsets
// and string-table references.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <unordered_map>
#include <stdexcept>

#include "core/math.h"

namespace r2c {

class Buffer {
public:
    size_t Size() const { return data_.size(); }
    size_t Tell() const { return data_.size(); }
    std::vector<uint8_t>& Data() { return data_; }
    const std::vector<uint8_t>& Data() const { return data_; }

    void Align(size_t a, uint8_t fill = 0) {
        while (data_.size() % a) data_.push_back(fill);
    }
    size_t Reserve(size_t n) {
        size_t at = data_.size();
        data_.resize(at + n, 0);
        return at;
    }
    void Bytes(const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        data_.insert(data_.end(), b, b + n);
    }
    void Append(const Buffer& o) { Bytes(o.data_.data(), o.data_.size()); }
    void Append(const std::vector<uint8_t>& o) { Bytes(o.data(), o.size()); }

    template <typename T>
    void Put(const T& v) {
        Bytes(&v, sizeof(T));
    }
    void U8(uint8_t v) { Put(v); }
    void I8(int8_t v) { Put(v); }
    void U16(uint16_t v) { Put(v); }
    void I16(int16_t v) { Put(v); }
    void I32(int32_t v) { Put(v); }
    void U32(uint32_t v) { Put(v); }
    void U64(uint64_t v) { Put(v); }
    void F32(float v) { Put(v); }
    void V3(const Vec3& v) { F32(v.x); F32(v.y); F32(v.z); }
    void V2(const Vec2& v) { F32(v.x); F32(v.y); }
    void Q(const Quat& q) { F32(q.x); F32(q.y); F32(q.z); F32(q.w); }
    void RE(const RadianEuler& e) { F32(e.x); F32(e.y); F32(e.z); }
    void M34(const Mat3x4& m) {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++) F32(m.m[i][j]);
    }
    void Zero(size_t n) { data_.insert(data_.end(), n, 0); }
    void FixedString(const std::string& s, size_t n) {
        size_t at = Reserve(n);
        std::memcpy(&data_[at], s.data(), std::min(s.size(), n - 1));
    }
    void CString(const std::string& s) {
        Bytes(s.data(), s.size());
        U8(0);
    }

    template <typename T>
    void PatchAt(size_t at, const T& v) {
        if (at + sizeof(T) > data_.size()) throw std::runtime_error("buffer patch out of range");
        std::memcpy(&data_[at], &v, sizeof(T));
    }
    void PatchI32(size_t at, int32_t v) { PatchAt(at, v); }
    template <typename T>
    T ReadAt(size_t at) const {
        T v;
        std::memcpy(&v, &data_[at], sizeof(T));
        return v;
    }

private:
    std::vector<uint8_t> data_;
};

// String table: fields that reference a string are recorded as
// (field position, base position the offset is relative to, string).
// Write() appends the deduplicated strings and patches every field.
class StringTable {
public:
    void Add(size_t fieldPos, size_t basePos, const std::string& s) { refs_.push_back({fieldPos, basePos, s}); }
    // Writes the table at the current end of buf and patches all recorded references.
    void Write(Buffer& buf) {
        std::unordered_map<std::string, size_t> where;
        // Respawn's studiomdl writes strings in first-reference order, deduplicated,
        // after an empty string that always opens the table.
        where[""] = buf.Tell();
        buf.CString("");
        for (auto& r : refs_) {
            if (where.count(r.s)) continue;
            where[r.s] = buf.Tell();
            buf.CString(r.s);
        }
        for (auto& r : refs_) {
            int64_t rel = static_cast<int64_t>(where[r.s]) - static_cast<int64_t>(r.base);
            buf.PatchI32(r.field, static_cast<int32_t>(rel));
        }
    }
    bool Empty() const { return refs_.empty(); }

private:
    struct Ref {
        size_t field, base;
        std::string s;
    };
    std::vector<Ref> refs_;
};

}  // namespace r2c
