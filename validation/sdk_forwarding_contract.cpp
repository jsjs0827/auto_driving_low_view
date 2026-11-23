// Isolated forwarding contract: not an inference or hardware test.
#include "inference_session_api.hpp"
#include "full_pipeline_api.hpp"
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>
struct Token {
    std::uintptr_t value;
    template<class T> Token(T v) : value(encode(v)) {}
    template<class T> static std::uintptr_t encode(T v) {
        if constexpr(std::is_pointer_v<T>) return reinterpret_cast<std::uintptr_t>(v);
        else return static_cast<std::uintptr_t>(v);
    }
    bool operator==(Token rhs)const{return value==rhs.value;}
};
std::string last_symbol;
std::vector<Token> last_arguments;
void require(bool condition,const char* description){
    if(!condition){std::cerr<<last_symbol<<": "<<description<<'\n';std::exit(1);}
}
std::uint32_t temporal_callback(void*,const psdk_temporal_view*,psdk_status*){return 0;}
namespace perception::sdk_internal {
std::uint32_t impl_psdk_full_push_motion_samples(psdk_full_engine*,const psdk_motion_sample*,std::uint32_t,psdk_status*) { return 0; }
std::uint32_t impl_psdk_full_result_get_slip_view(const psdk_full_result*,psdk_slip_view*,psdk_status*) { return 0; }
uint32_t impl_psdk_abi_version(void) { last_symbol="psdk_abi_version"; last_arguments={}; return 16640; }
uint32_t impl_psdk_build_capabilities(void) { last_symbol="psdk_build_capabilities"; last_arguments={}; return 16641; }
uint32_t impl_psdk_create(const psdk_config* config,psdk_engine** engine,psdk_status* status) { last_symbol="psdk_create"; last_arguments={config, engine, status}; if(engine)*engine=reinterpret_cast<psdk_engine*>(std::uintptr_t{32770}); if(status)status->code=16642; return 16642; }
uint32_t impl_psdk_process(psdk_engine* engine,const psdk_frame* frame,psdk_result** result,psdk_status* status) { last_symbol="psdk_process"; last_arguments={engine, frame, result, status}; if(result)*result=reinterpret_cast<psdk_result*>(std::uintptr_t{32771}); if(status)status->code=16643; return 16643; }
uint32_t impl_psdk_result_get_view(const psdk_result* result,psdk_result_view* view,psdk_status* status) { last_symbol="psdk_result_get_view"; last_arguments={result, view, status}; if(status)status->code=16644; return 16644; }
uint32_t impl_psdk_save_png(const psdk_result* result,const psdk_frame* frame,const char* path,psdk_status* status) { last_symbol="psdk_save_png"; last_arguments={result, frame, path, status}; if(status)status->code=16645; return 16645; }
void impl_psdk_result_release(psdk_result* result) { last_symbol="psdk_result_release"; last_arguments={result}; }
uint32_t impl_psdk_destroy(psdk_engine** engine,psdk_status* status) { last_symbol="psdk_destroy"; last_arguments={engine, status}; if(engine)*engine=reinterpret_cast<psdk_engine*>(std::uintptr_t{32775}); if(status)status->code=16647; return 16647; }
std::uint32_t impl_psdk_full_result_get_algorithms_json(const psdk_full_result* result,const char** json,psdk_status* error) {last_symbol="psdk_full_result_get_algorithms_json";last_arguments={result,json,error};if(json)*json="{}";if(error)error->code=16659;return 16659;}
std::uint32_t impl_psdk_full_capabilities(void) { last_symbol="psdk_full_capabilities"; last_arguments={}; return 16648; }
std::uint32_t impl_psdk_full_create(const psdk_full_config* config,psdk_full_engine** engine,psdk_status* error) { last_symbol="psdk_full_create"; last_arguments={config, engine, error}; if(engine)*engine=reinterpret_cast<psdk_full_engine*>(std::uintptr_t{32777}); if(error)error->code=16649; return 16649; }
std::uint32_t impl_psdk_full_process(psdk_full_engine* e,const psdk_full_frame* frame,psdk_full_result** result,psdk_status* error) { last_symbol="psdk_full_process"; last_arguments={e, frame, result, error}; if(result)*result=reinterpret_cast<psdk_full_result*>(std::uintptr_t{32778}); if(error)error->code=16650; return 16650; }
std::uint32_t impl_psdk_full_result_get_view(const psdk_full_result* result,psdk_full_result_view* view,psdk_status* error) { last_symbol="psdk_full_result_get_view"; last_arguments={result, view, error}; if(error)error->code=16651; return 16651; }
std::uint32_t impl_psdk_full_command(psdk_full_engine* e,std::int32_t command,psdk_status* error) { last_symbol="psdk_full_command"; last_arguments={e, command, error}; if(error)error->code=16652; return 16652; }
std::uint32_t impl_psdk_full_save_result(const psdk_full_result* result,const char* directory,psdk_status* error) { last_symbol="psdk_full_save_result"; last_arguments={result, directory, error}; if(error)error->code=16653; return 16653; }
void impl_psdk_full_result_release(psdk_full_result* result) { last_symbol="psdk_full_result_release"; last_arguments={result}; }
std::uint32_t impl_psdk_full_result_get_temporal_view(const psdk_full_result* result,psdk_temporal_view* view,psdk_status* error) { last_symbol="psdk_full_result_get_temporal_view"; last_arguments={result, view, error}; if(error)error->code=16655; return 16655; }
std::uint32_t impl_psdk_full_set_temporal_callback(psdk_full_engine* e,psdk_temporal_callback callback,void* userdata,psdk_status* error) { last_symbol="psdk_full_set_temporal_callback"; last_arguments={e, callback, userdata, error}; if(error)error->code=16656; return 16656; }
std::uint32_t impl_psdk_full_reset_history(psdk_full_engine* e,psdk_status* error) { last_symbol="psdk_full_reset_history"; last_arguments={e, error}; if(error)error->code=16657; return 16657; }
std::uint32_t impl_psdk_full_destroy(psdk_full_engine** engine,psdk_status* error) { last_symbol="psdk_full_destroy"; last_arguments={engine, error}; if(engine)*engine=reinterpret_cast<psdk_full_engine*>(std::uintptr_t{32786}); if(error)error->code=16658; return 16658; }
}
int main(){
    { require(psdk_abi_version()==16640,"return value"); require(last_symbol=="psdk_abi_version","dispatch"); require(last_arguments==std::vector<Token>{},"argument identity/order"); }
    { require(psdk_build_capabilities()==16641,"return value"); require(last_symbol=="psdk_build_capabilities","dispatch"); require(last_arguments==std::vector<Token>{},"argument identity/order"); }
    { psdk_config value_0{};const psdk_config* arg_0=&value_0; psdk_engine* value_1=nullptr;auto arg_1=&value_1; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_create(arg_0, arg_1, arg_2)==16642,"return value"); require(last_symbol=="psdk_create","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(reinterpret_cast<std::uintptr_t>(value_1)==32770,"output handle"); require(arg_2->code==16642,"status return"); }
    { psdk_engine* arg_0=reinterpret_cast<psdk_engine*>(std::uintptr_t{4096}); psdk_frame value_1{};const psdk_frame* arg_1=&value_1; psdk_result* value_2=nullptr;auto arg_2=&value_2; psdk_status value_3{};psdk_status* arg_3=&value_3; require(psdk_process(arg_0, arg_1, arg_2, arg_3)==16643,"return value"); require(last_symbol=="psdk_process","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2, arg_3},"argument identity/order"); require(reinterpret_cast<std::uintptr_t>(value_2)==32771,"output handle"); require(arg_3->code==16643,"status return"); }
    { const psdk_result* arg_0=reinterpret_cast<const psdk_result*>(std::uintptr_t{4096}); psdk_result_view value_1{};psdk_result_view* arg_1=&value_1; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_result_get_view(arg_0, arg_1, arg_2)==16644,"return value"); require(last_symbol=="psdk_result_get_view","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(arg_2->code==16644,"status return"); }
    { const psdk_result* arg_0=reinterpret_cast<const psdk_result*>(std::uintptr_t{4096}); psdk_frame value_1{};const psdk_frame* arg_1=&value_1; const char* arg_2="recording/frame.png"; psdk_status value_3{};psdk_status* arg_3=&value_3; require(psdk_save_png(arg_0, arg_1, arg_2, arg_3)==16645,"return value"); require(last_symbol=="psdk_save_png","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2, arg_3},"argument identity/order"); require(arg_3->code==16645,"status return"); }
    { psdk_result* arg_0=reinterpret_cast<psdk_result*>(std::uintptr_t{4096}); psdk_result_release(arg_0); require(last_symbol=="psdk_result_release","dispatch"); require(last_arguments==std::vector<Token>{arg_0},"argument identity/order"); }
    { psdk_engine* value_0=nullptr;auto arg_0=&value_0; psdk_status value_1{};psdk_status* arg_1=&value_1; require(psdk_destroy(arg_0, arg_1)==16647,"return value"); require(last_symbol=="psdk_destroy","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1},"argument identity/order"); require(reinterpret_cast<std::uintptr_t>(value_0)==32775,"output handle"); require(arg_1->code==16647,"status return"); }
    { require(psdk_full_capabilities()==16648,"return value"); require(last_symbol=="psdk_full_capabilities","dispatch"); require(last_arguments==std::vector<Token>{},"argument identity/order"); }
    { psdk_full_config value_0{};const psdk_full_config* arg_0=&value_0; psdk_full_engine* value_1=nullptr;auto arg_1=&value_1; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_full_create(arg_0, arg_1, arg_2)==16649,"return value"); require(last_symbol=="psdk_full_create","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(reinterpret_cast<std::uintptr_t>(value_1)==32777,"output handle"); require(arg_2->code==16649,"status return"); }
    { psdk_full_engine* arg_0=reinterpret_cast<psdk_full_engine*>(std::uintptr_t{4096}); psdk_full_frame value_1{};const psdk_full_frame* arg_1=&value_1; psdk_full_result* value_2=nullptr;auto arg_2=&value_2; psdk_status value_3{};psdk_status* arg_3=&value_3; require(psdk_full_process(arg_0, arg_1, arg_2, arg_3)==16650,"return value"); require(last_symbol=="psdk_full_process","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2, arg_3},"argument identity/order"); require(reinterpret_cast<std::uintptr_t>(value_2)==32778,"output handle"); require(arg_3->code==16650,"status return"); }
    { const psdk_full_result* arg_0=reinterpret_cast<const psdk_full_result*>(std::uintptr_t{4096}); psdk_full_result_view value_1{};psdk_full_result_view* arg_1=&value_1; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_full_result_get_view(arg_0, arg_1, arg_2)==16651,"return value"); require(last_symbol=="psdk_full_result_get_view","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(arg_2->code==16651,"status return"); }
    { psdk_full_engine* arg_0=reinterpret_cast<psdk_full_engine*>(std::uintptr_t{4096}); std::int32_t arg_1=21; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_full_command(arg_0, arg_1, arg_2)==16652,"return value"); require(last_symbol=="psdk_full_command","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(arg_2->code==16652,"status return"); }
    { const psdk_full_result* arg_0=reinterpret_cast<const psdk_full_result*>(std::uintptr_t{4096}); const char* arg_1="recording/frame.png"; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_full_save_result(arg_0, arg_1, arg_2)==16653,"return value"); require(last_symbol=="psdk_full_save_result","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(arg_2->code==16653,"status return"); }
    { psdk_full_result* arg_0=reinterpret_cast<psdk_full_result*>(std::uintptr_t{4096}); psdk_full_result_release(arg_0); require(last_symbol=="psdk_full_result_release","dispatch"); require(last_arguments==std::vector<Token>{arg_0},"argument identity/order"); }
    { const psdk_full_result* arg_0=reinterpret_cast<const psdk_full_result*>(std::uintptr_t{4096}); psdk_temporal_view value_1{};psdk_temporal_view* arg_1=&value_1; psdk_status value_2{};psdk_status* arg_2=&value_2; require(psdk_full_result_get_temporal_view(arg_0, arg_1, arg_2)==16655,"return value"); require(last_symbol=="psdk_full_result_get_temporal_view","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2},"argument identity/order"); require(arg_2->code==16655,"status return"); }
    { psdk_full_engine* arg_0=reinterpret_cast<psdk_full_engine*>(std::uintptr_t{4096}); psdk_temporal_callback arg_1=&temporal_callback; void* arg_2=reinterpret_cast<void*>(std::uintptr_t{321}); psdk_status value_3{};psdk_status* arg_3=&value_3; require(psdk_full_set_temporal_callback(arg_0, arg_1, arg_2, arg_3)==16656,"return value"); require(last_symbol=="psdk_full_set_temporal_callback","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1, arg_2, arg_3},"argument identity/order"); require(arg_3->code==16656,"status return"); }
    { psdk_full_engine* arg_0=reinterpret_cast<psdk_full_engine*>(std::uintptr_t{4096}); psdk_status value_1{};psdk_status* arg_1=&value_1; require(psdk_full_reset_history(arg_0, arg_1)==16657,"return value"); require(last_symbol=="psdk_full_reset_history","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1},"argument identity/order"); require(arg_1->code==16657,"status return"); }
    { psdk_full_engine* value_0=nullptr;auto arg_0=&value_0; psdk_status value_1{};psdk_status* arg_1=&value_1; require(psdk_full_destroy(arg_0, arg_1)==16658,"return value"); require(last_symbol=="psdk_full_destroy","dispatch"); require(last_arguments==std::vector<Token>{arg_0, arg_1},"argument identity/order"); require(reinterpret_cast<std::uintptr_t>(value_0)==32786,"output handle"); require(arg_1->code==16658,"status return"); }
    {const auto* result=reinterpret_cast<const psdk_full_result*>(std::uintptr_t{4096});const char* json=nullptr;psdk_status error{};
      require(psdk_full_result_get_algorithms_json(result,&json,&error)==16659,"algorithm getter return");
      require(last_arguments==std::vector<Token>{result,&json,&error},"algorithm getter argument identity/order");
      require(json&&std::string(json)=="{}"&&error.code==16659,"algorithm getter output/status");}
    std::cout<<"20 public C ABI forwarding signatures, all arguments/statuses/output handles preserved\n";
}
