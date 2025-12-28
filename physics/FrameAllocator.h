#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <cstdint>
#include <mutex>

class FrameAllocator {
public:
    FrameAllocator(size_t blockSize = 1024 * 1024) : defaultBlockSize(blockSize) {
        blocks.reserve(16);
    }

    void* allocate(size_t size, size_t align = 16) {
        size_t adjustment = (align - (currentOffset & (align - 1))) & (align - 1);
        
        if (blockIndex >= blocks.size() || currentOffset + adjustment + size > blocks[blockIndex].size) {
            allocateNewBlock(size + adjustment);
            adjustment = (align - (currentOffset & (align - 1))) & (align - 1);
        }

        uint8_t* ptr = blocks[blockIndex].ptr.get() + currentOffset + adjustment;
        currentOffset += adjustment + size;
        return ptr;
    }

    template<typename T, typename... Args>
    T* newObject(Args&&... args) {
        void* mem = allocate(sizeof(T), alignof(T));
        return new(mem) T(std::forward<Args>(args)...);
    }

    void reset() {
        blockIndex = 0;
        currentOffset = 0;
    }

private:
    struct Block {
        std::unique_ptr<uint8_t[]> ptr;
        size_t size;
    };

    void allocateNewBlock(size_t required) {
        if (blockIndex < blocks.size()) {
            blockIndex++;
        }
        
        if (blockIndex >= blocks.size()) {
            size_t sz = std::max(defaultBlockSize, required);
            blocks.push_back({std::make_unique<uint8_t[]>(sz), sz});
        } else if (blocks[blockIndex].size < required) {
             size_t sz = std::max(defaultBlockSize, required);
             blocks[blockIndex] = {std::make_unique<uint8_t[]>(sz), sz};
        }
        currentOffset = 0;
    }

    std::vector<Block> blocks;
    size_t defaultBlockSize;
    size_t blockIndex = 0;
    size_t currentOffset = 0;
};

class ThreadSafeFrameAllocator {
public:
    ThreadSafeFrameAllocator(size_t blockSize = 1024 * 1024) : allocator(blockSize) {}

    void* allocate(size_t size, size_t align = 16) {
        std::lock_guard<std::mutex> lock(mutex);
        return allocator.allocate(size, align);
    }

    template<typename T, typename... Args>
    T* newObject(Args&&... args) {
        std::lock_guard<std::mutex> lock(mutex);
        return allocator.newObject<T>(std::forward<Args>(args)...);
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex);
        allocator.reset();
    }

private:
    FrameAllocator allocator;
    std::mutex mutex;
};
