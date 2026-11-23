#include "perception_ros_so_demo/latest_frame.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <future>
#include <string>
#include <thread>

int main() {
    using ros_so_demo::ExactFrames;
    ExactFrames<std::string,4> sync(15,2);
    assert(!sync.push(0,100,"left100"));
    assert(!sync.push(1,101,"right101")); // Near stamps must never pair.
    assert(!sync.push(1,100,"right100"));
    assert(!sync.push(2,100,"left_info100"));
    auto complete=sync.push(3,100,"right_info100");
    assert(complete&&complete->stamp==100&&*complete->values[0]=="left100");
    assert(!sync.push(0,100,"duplicate"));
    assert(sync.stats().stale_messages==1);
    assert(!sync.push(0,102,"left102"));
    assert(!sync.push(0,103,"left103"));
    assert(sync.stats().pending==2&&sync.stats().evicted==1); // Incomplete101 evicted.
    assert(!sync.push(1,103,"right103"));
    assert(!sync.push(2,103,"left_info103"));
    complete=sync.push(3,103,"right_info103");
    assert(complete&&sync.stats().evicted==2&&sync.stats().pending==0);
    assert(!sync.push(1,102,"late right102"));
    assert(sync.stats().stale_messages==2&&sync.stats().matched==2);
    // Distinct left/right optical frame labels are retained, not compared.
    ExactFrames<std::string,2> pair(3,4);
    pair.push(0,500,"left_optical");
    auto labels=pair.push(1,500,"right_optical");
    assert(labels&&*labels->values[0]!=*labels->values[1]);

    ros_so_demo::LatestFrame<int> mailbox;
    assert(mailbox.put(1));assert(mailbox.hasPending());int active=0;assert(mailbox.take(active)&&active==1);assert(!mailbox.hasPending());
    assert(mailbox.put(2));assert(mailbox.put(3));assert(mailbox.put(4));
    assert(mailbox.replaced()==2);assert(mailbox.take(active)&&active==4);
    // Shutdown wakes a blocked inference worker and forbids future work.
    auto waiting=std::async(std::launch::async,[&]{int v=0;return mailbox.take(v);});
    assert(waiting.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout);
    mailbox.close();assert(!waiting.get());assert(!mailbox.put(5));
    ros_so_demo::LatestFrame<int> discard;
    discard.put(8);discard.close();assert(discard.replaced()==1&&!discard.take(active));
}
