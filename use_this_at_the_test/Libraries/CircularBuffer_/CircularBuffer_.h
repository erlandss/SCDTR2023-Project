#ifndef BUFFER_H
#define BUFFER_H

template<typename T>
class CircularBuffer {
private:
    T* buffer_;

public:
    int head_;
    int tail_;
    int size_;
    int capacity_;

    CircularBuffer(int capacity) {
        buffer_ = new T[capacity];
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        capacity_ = capacity;
    }

    // Destructor to free dynamically allocated memory
    ~CircularBuffer() {
        delete[] buffer_;
    }

    // Add an element to the buffer
    void add(const T& value) {
        buffer_[tail_] = value;
        tail_ = (tail_ + 1) % capacity_;
        if (size_ < capacity_) {
            size_++;
        } else {
            head_ = (head_ + 1) % capacity_;
        }
    }

    // Get reference to the first element in the buffer
    const T* get_buffer() const {
        return buffer_;
    }

    const T& get_newest() const {
    int index = (tail_ + capacity_ - 1) % capacity_;
    return buffer_[index];
}    
};

#endif
