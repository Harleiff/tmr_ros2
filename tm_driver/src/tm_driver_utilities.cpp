#include "../include/tm_driver/tm_driver_utilities.h"

// DispatchQueue

void DispatchQueue::put(std::function<void()> op) {
    std::lock_guard<std::mutex> guard(qlock);
    cmdQueue.push(std::move(op));
    empty.notify_one();
}

// Returns true and fills 'out' if a real job was dequeued.
// Returns false if a null (shutdown sentinel) was dequeued — caller should stop.
bool DispatchQueue::take(std::function<void()> &out) {
    std::unique_lock<std::mutex> lock(qlock);
    empty.wait(lock, [&] { return !cmdQueue.empty(); });

    std::function<void()> op = std::move(cmdQueue.front());
    cmdQueue.pop();

    if (!op) {
        // Null function = shutdown sentinel
        return false;
    }
    out = std::move(op);
    return true;
}

// ActiveObject

ActiveObject::ActiveObject() : val(0), done(false) {
    runnable = new std::thread(&ActiveObject::run, this);
}

ActiveObject::~ActiveObject() {
    done = true;
    // Push a null (sentinel) to unblock the waiting take()
    dispatchQueue.put(std::function<void()>());
    if (runnable->joinable()) {
        runnable->join();
    }
    delete runnable;
}

void ActiveObject::run() {
    while (!done) {
        std::function<void()> func;
        if (!dispatchQueue.take(func)) {
            // Received shutdown sentinel
            break;
        }
        func();
    }
}

void ActiveObject::set_function(std::function<void()> func) {
    dispatchQueue.put(std::move(func));
}