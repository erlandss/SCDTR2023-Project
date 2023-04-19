#ifndef QUEUE_H
#define QUEUE_H

template<typename T>
class Queue {
private:
    template<typename U>
    struct Node {
        U data;
        Node* next;
        Node(U data) : data(data), next(nullptr) {}
        ~Node() { delete next; }
    };

    Node<T>* head;
    Node<T>* tail;

public:
    Queue() : head(nullptr), tail(nullptr) {}

    ~Queue() {
        delete head;
    }

    void add(T value) {
        Node<T>* newNode = new Node<T>(value);
        if (tail != nullptr) {
            tail->next = newNode;
        }
        tail = newNode;
        if (head == nullptr) {
            head = tail;
        }
    }

    T pop() {
        if (head == nullptr) {
            //Shit, do something?
            return '\0';
        }
        T value = head->data;
        Node<T>* tmp = head;
        head = head->next;
        if (head == nullptr) {
            tail = nullptr;
        }
        tmp->next = nullptr; // Break link to the rest of the list
        delete tmp;
        return value;
    }

    bool is_empty() const {
        return head == nullptr;
    }
};

#endif