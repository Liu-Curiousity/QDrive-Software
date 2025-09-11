/**
 * @brief 		CharCircularQueue库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2025/6/22
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于2025/6/22
 * */

#pragma once

class CharCircularQueue {
public:
    // 构造函数：动态指定容量
    explicit CharCircularQueue(const int capacity) :
        size(capacity), head(0), tail(0), count(0) {
        buf = new char[size];
    }

    ~CharCircularQueue() {
        delete[] buf;
    }

    [[nodiscard]] bool isEmpty() const { return count == 0; }
    [[nodiscard]] bool isFull() const { return count == size; }

    bool enqueue(char c) {
        if (isFull()) return false;
        buf[tail] = c;
        tail = (tail + 1) % size;
        ++count;
        return true;
    }

    bool dequeue(char& c) {
        if (isEmpty()) return false;
        c = buf[head];
        head = (head + 1) % size;
        --count;
        return true;
    }

    [[nodiscard]] char front() const { return buf[head]; }
    [[nodiscard]] int Size() const { return size; }
    [[nodiscard]] int currentSize() const { return count; }

private:
    char *buf;
    int size;
    int head;
    int tail;
    int count;
};
