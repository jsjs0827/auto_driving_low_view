#pragma once
#include <array>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace ros_so_demo {
// Exact capture identity only: receive time must never stand in for a stamp.
// One entry per capture bounds memory across all subscribed topics together.
template<class T,std::size_t N> class ExactFrames {
public:
    struct Match {std::uint64_t stamp=0;std::array<std::optional<T>,N> values;};
    struct Stats {std::uint64_t matched=0,evicted=0,stale_messages=0;std::size_t pending=0;};
private:
    std::map<std::uint64_t,Match> entries_;
    std::uint64_t required_,last_=0;
    std::size_t capacity_;
    Stats stats_;
public:
    ExactFrames(std::uint64_t required,std::size_t capacity):required_(required),capacity_(capacity) {
        if(!required||!capacity||N>63||required>>N)throw std::invalid_argument("Invalid exact frame synchronization contract");
    }
    // Called from a mutually exclusive ROS callback group, not inference.
    std::optional<Match> push(std::size_t slot,std::uint64_t stamp,T value) {
        if(slot>=N||!stamp)throw std::invalid_argument("Invalid exact frame slot/stamp");
        if(stamp<=last_){++stats_.stale_messages;return {};}
        auto& entry=entries_[stamp];entry.stamp=stamp;entry.values[slot]=std::move(value);
        bool complete=true;
        for(std::size_t i=0;i<N;++i)if((required_&(std::uint64_t(1)<<i))&&!entry.values[i])complete=false;
        if(complete) {
            auto result=std::move(entry);
            auto end=entries_.upper_bound(stamp);
            for(auto it=entries_.begin();it!=end;++it)if(it->first!=stamp)++stats_.evicted;
            entries_.erase(entries_.begin(),end);last_=stamp;++stats_.matched;
            return result;
        }
        while(entries_.size()>capacity_){entries_.erase(entries_.begin());++stats_.evicted;}
        return {};
    }
    Stats stats()const {auto out=stats_;out.pending=entries_.size();return out;}
    std::uint64_t lastStamp()const{return last_;}
};

// Inference owns one active capture and at most one waiting capture. Replacing
// waiting work is intentional; live input cannot create an unbounded backlog.
template<class T> class LatestFrame {
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::optional<T> pending_;
    bool closed_=false;
    std::uint64_t replaced_=0;
public:
    bool put(T value) {
        {std::lock_guard<std::mutex> lock(mutex_);if(closed_)return false;
         if(pending_)++replaced_;pending_=std::move(value);}
        ready_.notify_one();return true;
    }
    bool take(T& out) {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock,[this]{return closed_||pending_.has_value();});
        if(closed_)return false;
        out=std::move(*pending_);pending_.reset();return true;
    }
    void close() {
        {std::lock_guard<std::mutex> lock(mutex_);closed_=true;
         if(pending_){++replaced_;pending_.reset();}}
        ready_.notify_all();
    }
    std::uint64_t replaced()const{std::lock_guard<std::mutex> lock(mutex_);return replaced_;}
    bool hasPending()const{std::lock_guard<std::mutex> lock(mutex_);return pending_.has_value();}
};
}
