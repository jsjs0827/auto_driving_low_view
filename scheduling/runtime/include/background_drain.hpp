#pragma once

#include "camera_source.hpp"
#include "original_input_contract.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace r27 {

// Consume/release SDK RGBD tuples while AI uses immutable frozen input.
// There is no AI preparation, inference queue or cross-frame pipeline here.
// stop()/destruction belong to the owning control thread; check() may also be
// called while acquisition runs. Counters may only be read after stop()/join.
class BackgroundDrain final {
    std::shared_ptr<demo_camera::Source> source_;
    std::shared_ptr<std::ostream> csv_;
    std::atomic<bool> stop_{false},failed_{false},finalized_{false};
    // Published once, before the release store to failed_; never replaced.
    std::exception_ptr error_;
    std::thread thread_;
    std::uint64_t count_=0,metadata_verified_=0,metadata_unverified_=0;

    static std::int64_t nowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    static std::shared_ptr<std::ostream> evidenceFile(const std::filesystem::path& path) {
        auto file=std::make_shared<std::ofstream>(path,std::ios::binary);
        if(!*file)throw std::runtime_error("Cannot create background frame evidence");
        return file;
    }
    static std::string csvString(const std::string& value) {
        std::string result="\"";
        for(char c:value){if(c=='\"')result+='\"';result+=c;}
        return result+'\"';
    }
    void publishFailure(std::exception_ptr error) noexcept {
        // Only the acquisition thread writes while running. The owner may
        // publish a flush failure only after joining that thread.
        if(!failed_.load(std::memory_order_relaxed)) {
            error_=std::move(error);
            failed_.store(true,std::memory_order_release);
        }
    }
    void requireFinalized() const {
        if(!finalized_.load(std::memory_order_acquire))
            throw std::logic_error("Background counters require stop/join first");
    }
    void run() noexcept {
        try {
            while(!stop_.load(std::memory_order_acquire)) {
                auto frame=source_->next();
                if(!frame.image)throw std::runtime_error("Background source returned no image");
                r26::validateInput(frame.image->view());
                // Missing metadata is explicitly unverified, never a pass.
                // A zero frame ID is legal. Source's triplet audit establishes
                // the ID/shape relationship; this additionally needs timestamps.
                const bool verified=frame.sdk_rgbd_triplet_available &&
                    frame.sdk_rgbd_triplet_valid && frame.acquisition.sdk_timestamp &&
                    frame.info.timestamp_ns!=0 && frame.sdk_right_timestamp_ns!=0 &&
                    frame.sdk_disparity_timestamp_ns!=0;
                std::string issue=frame.sdk_rgbd_triplet_issue;
                if(!verified && issue.empty())issue="background_metadata_unverified";
                *csv_<<nowNs()<<','<<frame.info.id<<','<<frame.info.timestamp_ns<<','
                     <<frame.acquisition.sdk_timestamp<<','<<frame.sdk_rgbd_triplet_available<<','
                     <<frame.sdk_rgbd_triplet_valid<<','<<frame.sdk_disparity_frame_id<<','
                     <<frame.sdk_disparity_timestamp_ns<<','<<csvString(issue)<<','<<verified<<','
                     <<frame.sdk_color_pair_valid<<','<<csvString(frame.sdk_color_pair_issue)<<'\n';
                if(!*csv_)throw std::runtime_error("Background frame evidence write failed");
                ++count_;
                if(verified)++metadata_verified_;else ++metadata_unverified_;
            }
        } catch(...) {
            publishFailure(std::current_exception());
        }
    }

public:
    BackgroundDrain(std::shared_ptr<demo_camera::Source> source,
                    const std::filesystem::path& path)
        :BackgroundDrain(std::move(source),evidenceFile(path)) {}

    // Stream ownership also permits deterministic write/flush fault tests;
    // callers must not access the stream until stop() has joined the writer.
    BackgroundDrain(std::shared_ptr<demo_camera::Source> source,
                    std::shared_ptr<std::ostream> evidence)
        :source_(std::move(source)),csv_(std::move(evidence)) {
        if(!source_)throw std::invalid_argument("Background source is required");
        if(!csv_ || !*csv_)throw std::runtime_error("Cannot create background frame evidence");
        *csv_<<"host_ns,frame_id,sdk_timestamp_ns,sdk_timestamp_valid,sdk_rgbd_triplet_available,sdk_rgbd_triplet_valid,sdk_disparity_frame_id,sdk_disparity_timestamp_ns,sdk_rgbd_triplet_issue,drain_triplet_verified,sdk_color_pair_valid,sdk_color_pair_issue\n";
        if(!*csv_)throw std::runtime_error("Background frame evidence header write failed");
        thread_=std::thread([this]{run();});
    }
    BackgroundDrain(const BackgroundDrain&)=delete;
    BackgroundDrain& operator=(const BackgroundDrain&)=delete;
    BackgroundDrain(BackgroundDrain&&)=delete;
    BackgroundDrain& operator=(BackgroundDrain&&)=delete;
    ~BackgroundDrain(){stop();}

    // Source::next() has its own bounded acquisition timeout. Stop waits for
    // its in-flight call before destroying any Source or stream ownership.
    // Flush failures are published for check(); no destructor exception.
    void stop() noexcept {
        stop_.store(true,std::memory_order_release);
        if(thread_.joinable())thread_.join();
        if(finalized_.load(std::memory_order_acquire))return;
        try {
            csv_->flush();
            if(!*csv_)throw std::runtime_error("Background frame evidence flush failed");
        } catch(...) {
            publishFailure(std::current_exception());
        }
        finalized_.store(true,std::memory_order_release);
    }
    void check() const {
        // Never inspect ostream state while the acquisition thread writes it.
        if(failed_.load(std::memory_order_acquire))std::rethrow_exception(error_);
    }
    std::uint64_t count() const {requireFinalized();return count_;}
    std::uint64_t metadataVerifiedCount() const {requireFinalized();return metadata_verified_;}
    std::uint64_t metadataUnverifiedCount() const {requireFinalized();return metadata_unverified_;}
};

} // namespace r27
