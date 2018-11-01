#ifndef SIMPLE_SIGNAL_H
#define SIMPLE_SIGNAL_H

#include <memory>
#include <functional>
#include <vector>

namespace BT
{
/**
 * Super simple Signal/Slop implementation, AKA "Observable pattern".
 * The subscriber is active until it goes out of scope or Subscriber::reset() is called.
 */
template <typename... CallableArgs>
class Signal
{
  public:
    using CallableFunction = std::function<void(CallableArgs...)>;
    using Subscriber = std::shared_ptr<CallableFunction>;

    void notify(CallableArgs... args)
    {
        for (size_t i = 0; i < subscribers_.size();)
        {
            if (auto sub = subscribers_[i].lock())
            {
                (*sub)(args...);
                i++;
            }
            else
            {
                subscribers_.erase(subscribers_.begin() + i);
            }
        }
    }

    Subscriber subscribe(CallableFunction func)
    {
        Subscriber sub = std::make_shared<CallableFunction>(std::move(func));
        subscribers_.emplace_back(sub);
        return sub;
    }

  private:
    std::vector<std::weak_ptr<CallableFunction>> subscribers_;
};
}

#endif   // SIMPLE_SIGNAL_H
