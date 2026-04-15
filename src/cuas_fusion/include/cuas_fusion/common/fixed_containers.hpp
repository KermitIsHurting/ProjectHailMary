// @file fixed_containers.hpp
// @brief Pre-allocated fixed-capacity vector and map containers.
#pragma once

#include "cuas_fusion/common/fixed_types.hpp"
#include <array>
#include <cstddef>

namespace cuas {

template <typename T, uint32_t Capacity>
class FixedVector {
public:
    FixedVector() : size_(0) {}

    bool push_back(const T& item) {
        if (size_ >= Capacity) {
            return false;
        }
        data_[size_] = item;
        ++size_;
        return true;
    }

    void clear() { size_ = 0; }
    uint32_t size() const { return size_; }
    bool empty() const { return size_ == 0U; }
    static constexpr uint32_t capacity() { return Capacity; }

    T& operator[](uint32_t idx) { return data_[idx]; }
    const T& operator[](uint32_t idx) const { return data_[idx]; }

    T* begin() { return &data_[0]; }
    T* end() { return &data_[size_]; }
    const T* begin() const { return &data_[0]; }
    const T* end() const { return &data_[size_]; }

    T* data() { return &data_[0]; }
    const T* data() const { return &data_[0]; }

private:
    std::array<T, Capacity> data_;
    uint32_t size_ = 0;
};

template <typename Key, typename Value, uint32_t Capacity>
class FixedMap {
public:
    struct Entry {
        Key   key;
        Value value;
        bool  occupied = false;

        // Direct-init members accept Value types whose default ctor is explicit.
        Entry() : key(), value(), occupied(false) {}
    };

    FixedMap() = default;

    Value* find(const Key& key) {
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (entries_[i].occupied && entries_[i].key == key) {
                return &entries_[i].value;
            }
        }
        return nullptr;
    }

    const Value* find(const Key& key) const {
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (entries_[i].occupied && entries_[i].key == key) {
                return &entries_[i].value;
            }
        }
        return nullptr;
    }

    bool insert_or_assign(const Key& key, const Value& value) {
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (entries_[i].occupied && entries_[i].key == key) {
                entries_[i].value = value;
                return true;
            }
        }
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (!entries_[i].occupied) {
                entries_[i].key = key;
                entries_[i].value = value;
                entries_[i].occupied = true;
                return true;
            }
        }
        return false;
    }

    bool erase(const Key& key) {
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (entries_[i].occupied && entries_[i].key == key) {
                entries_[i].occupied = false;
                return true;
            }
        }
        return false;
    }

    template <typename Pred>
    void erase_if(Pred pred) {
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (entries_[i].occupied && pred(entries_[i].key, entries_[i].value)) {
                entries_[i].occupied = false;
            }
        }
    }

    uint32_t size() const {
        uint32_t count = 0;
        for (uint32_t i = 0; i < Capacity; ++i) {
            if (entries_[i].occupied) {
                ++count;
            }
        }
        return count;
    }

    void clear() {
        for (uint32_t i = 0; i < Capacity; ++i) {
            entries_[i].occupied = false;
        }
    }

    Entry* slots() { return entries_.data(); }
    const Entry* slots() const { return entries_.data(); }
    static constexpr uint32_t slot_count() { return Capacity; }

private:
    std::array<Entry, Capacity> entries_{};
};

}  // namespace cuas
