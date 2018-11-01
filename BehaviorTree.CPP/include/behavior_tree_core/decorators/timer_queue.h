#ifndef TIMERQUEUE_H
#define TIMERQUEUE_H

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <chrono>
#include <assert.h>

namespace BT
{
// http://www.crazygaze.com/blog/2016/03/24/portable-c-timer-queue/

namespace details
{
class Semaphore
{
  public:
    Semaphore(unsigned int count = 0) : m_count(count)
    {
    }

    void notify()
    {
        std::unique_lock<std::mutex> lock(m_mtx);
        m_count++;
        m_cv.notify_one();
    }

    void wait()
    {
        std::unique_lock<std::mutex> lock(m_mtx);
        m_cv.wait(lock, [this]() { return m_count > 0; });
        m_count--;
    }

    template <class Clock, class Duration>
    bool waitUntil(const std::chrono::time_point<Clock, Duration>& point)
    {
        std::unique_lock<std::mutex> lock(m_mtx);
        if (!m_cv.wait_until(lock, point, [this]() { return m_count > 0; }))
            return false;
        m_count--;
        return true;
    }

  private:
    std::mutex m_mtx;
    std::condition_variable m_cv;
    unsigned int m_count;
};
}

// Timer Queue
//
// Allows execution of handlers at a specified time in the future
// Guarantees:
//  - All handlers are executed ONCE, even if canceled (aborted parameter will
//be set to true)
//      - If TimerQueue is destroyed, it will cancel all handlers.
//  - Handlers are ALWAYS executed in the Timer Queue worker thread.
//  - Handlers execution order is NOT guaranteed
//
class TimerQueue
{
  public:
    TimerQueue()
    {
        m_th = std::thread([this] { run(); });
    }

    ~TimerQueue()
    {
        cancelAll();
        // Abusing the timer queue to trigger the shutdown.
        add(std::chrono::milliseconds(0), [this](bool) { m_finish = true; });
        m_th.join();
    }

    //! Adds a new timer
    // \return
    //  Returns the ID of the new timer. You can use this ID to cancel the
    // timer
    uint64_t add(std::chrono::milliseconds milliseconds, std::function<void(bool)> handler)
    {
        WorkItem item;
        item.end = Clock::now() + milliseconds;
        item.handler = std::move(handler);

        std::unique_lock<std::mutex> lk(m_mtx);
        uint64_t id = ++m_idcounter;
        item.id = id;
        m_items.push(std::move(item));
        lk.unlock();

        // Something changed, so wake up timer thread
        m_checkWork.notify();
        return id;
    }

    //! Cancels the specified timer
    // \return
    //  1 if the timer was cancelled.
    //  0 if you were too late to cancel (or the timer ID was never valid to
    // start with)
    size_t cancel(uint64_t id)
    {
        // Instead of removing the item from the container (thus breaking the
        // heap integrity), we set the item as having no handler, and put
        // that handler on a new item at the top for immediate execution
        // The timer thread will then ignore the original item, since it has no
        // handler.
        std::unique_lock<std::mutex> lk(m_mtx);
        for (auto&& item : m_items.getContainer())
        {
            if (item.id == id && item.handler)
            {
                WorkItem newItem;
                // Zero time, so it stays at the top for immediate execution
                newItem.end = Clock::time_point();
                newItem.id = 0;   // Means it is a canceled item
                // Move the handler from item to newitem.
                // Also, we need to manually set the handler to nullptr, since
                // the standard does not guarantee moving an std::function will
                // empty it. Some STL implementation will empty it, others will
                // not.
                newItem.handler = std::move(item.handler);
                item.handler = nullptr;
                m_items.push(std::move(newItem));

                lk.unlock();
                // Something changed, so wake up timer thread
                m_checkWork.notify();
                return 1;
            }
        }
        return 0;
    }

    //! Cancels all timers
    // \return
    //  The number of timers cancelled
    size_t cancelAll()
    {
        // Setting all "end" to 0 (for immediate execution) is ok,
        // since it maintains the heap integrity
        std::unique_lock<std::mutex> lk(m_mtx);
        for (auto&& item : m_items.getContainer())
        {
            if (item.id)
            {
                item.end = Clock::time_point();
                item.id = 0;
            }
        }
        auto ret = m_items.size();

        lk.unlock();
        m_checkWork.notify();
        return ret;
    }

  private:
    using Clock = std::chrono::steady_clock;
    TimerQueue(const TimerQueue&) = delete;
    TimerQueue& operator=(const TimerQueue&) = delete;

    void run()
    {
        while (!m_finish)
        {
            auto end = calcWaitTime();
            if (end.first)
            {
                // Timers found, so wait until it expires (or something else
                // changes)
                m_checkWork.waitUntil(end.second);
            }
            else
            {
                // No timers exist, so wait forever until something changes
                m_checkWork.wait();
            }

            // Check and execute as much work as possible, such as, all expired
            // timers
            checkWork();
        }

        // If we are shutting down, we should not have any items left,
        // since the shutdown cancels all items
        assert(m_items.size() == 0);
    }

    std::pair<bool, Clock::time_point> calcWaitTime()
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        while (m_items.size())
        {
            if (m_items.top().handler)
            {
                // Item present, so return the new wait time
                return std::make_pair(true, m_items.top().end);
            }
            else
            {
                // Discard empty handlers (they were cancelled)
                m_items.pop();
            }
        }

        // No items found, so return no wait time (causes the thread to wait
        // indefinitely)
        return std::make_pair(false, Clock::time_point());
    }

    void checkWork()
    {
        std::unique_lock<std::mutex> lk(m_mtx);
        while (m_items.size() && m_items.top().end <= Clock::now())
        {
            WorkItem item(std::move(m_items.top()));
            m_items.pop();

            lk.unlock();
            if (item.handler)
                item.handler(item.id == 0);
            lk.lock();
        }
    }

    details::Semaphore m_checkWork;
    std::thread m_th;
    bool m_finish = false;
    uint64_t m_idcounter = 0;

    struct WorkItem
    {
        Clock::time_point end;
        uint64_t id;   // id==0 means it was cancelled
        std::function<void(bool)> handler;
        bool operator>(const WorkItem& other) const
        {
            return end > other.end;
        }
    };

    std::mutex m_mtx;
    // Inheriting from priority_queue, so we can access the internal container
    class Queue
      : public std::priority_queue<WorkItem, std::vector<WorkItem>, std::greater<WorkItem>>
    {
      public:
        std::vector<WorkItem>& getContainer()
        {
            return this->c;
        }
    } m_items;
};
}

#endif   // TIMERQUEUE_H
