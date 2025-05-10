#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <list>
#include <vector>
#include <condition_variable>

#include <opus_encoder.h> //音频编解码
#include <opus_decoder.h>
#include <opus_resampler.h> //音频重采样

#include "protocol.h" //通信协议层 websocket mqtt
#include "ota.h" //固件升级
#include "background_task.h" //后台任务

#if CONFIG_USE_WAKE_WORD_DETECT
#include "wake_word_detect.h" //唤醒词检测
#endif
#if CONFIG_USE_AUDIO_PROCESSOR
#include "audio_processor.h" //音频处理
#endif

#define SCHEDULE_EVENT (1 << 0)
#define AUDIO_INPUT_READY_EVENT (1 << 1)
#define AUDIO_OUTPUT_READY_EVENT (1 << 2)
#define CHECK_NEW_VERSION_DONE_EVENT (1 << 3)

// 设备状态转移
enum DeviceState {
    kDeviceStateUnknown, // 未知
    kDeviceStateStarting, // 启动中
    kDeviceStateWifiConfiguring, // 配网中
    kDeviceStateIdle, // 空闲
    kDeviceStateConnecting, // 连接中
    kDeviceStateListening, // 聆听中
    kDeviceStateSpeaking, // 说话中
    kDeviceStateUpgrading, // 升级中
    kDeviceStateActivating, // 激活中
    kDeviceStateFatalError // 致命错误
};

#define OPUS_FRAME_DURATION_MS 60 // 60ms 一个音频帧

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void Start();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return voice_detected_; } 
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void UpdateIotStates();
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    void PlaySound(const std::string_view& sound);
    bool CanEnterSleepMode();

private:
    Application();
    ~Application();

#if CONFIG_USE_WAKE_WORD_DETECT
    WakeWordDetect wake_word_detect_; // 唤醒词检测功能
#endif
#if CONFIG_USE_AUDIO_PROCESSOR
    AudioProcessor audio_processor_; // 音频处理功能(降噪, 回声消除)
#endif
    Ota ota_; // 固件升级功能
    std::mutex mutex_; // 互斥锁
    std::list<std::function<void()>> main_tasks_; // 主任务列表
    std::unique_ptr<Protocol> protocol_; // 协议层
    EventGroupHandle_t event_group_ = nullptr; // 事件组
    esp_timer_handle_t clock_timer_handle_ = nullptr; // 时钟定时器
    volatile DeviceState device_state_ = kDeviceStateUnknown; // 设备状态
    ListeningMode listening_mode_ = kListeningModeAutoStop; // 聆听模式
#if CONFIG_USE_REALTIME_CHAT
    bool realtime_chat_enabled_ = true; // 实时聊天功能
#else
    bool realtime_chat_enabled_ = false; // 实时聊天功能    
#endif
    bool aborted_ = false;
    bool voice_detected_ = false;
    bool busy_decoding_audio_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;

    // Audio encode / decode
    TaskHandle_t audio_loop_task_handle_ = nullptr;
    BackgroundTask* background_task_ = nullptr;
    std::chrono::steady_clock::time_point last_output_time_;
    std::list<std::vector<uint8_t>> audio_decode_queue_;
    std::condition_variable audio_decode_cv_;

    std::unique_ptr<OpusEncoderWrapper> opus_encoder_;
    std::unique_ptr<OpusDecoderWrapper> opus_decoder_;

    OpusResampler input_resampler_;
    OpusResampler reference_resampler_;
    OpusResampler output_resampler_;

    void MainEventLoop();
    void OnAudioInput();
    void OnAudioOutput();
    void ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples);
    void ResetDecoder();
    void SetDecodeSampleRate(int sample_rate, int frame_duration);
    void CheckNewVersion();
    void ShowActivationCode();
    void OnClockTimer();
    void SetListeningMode(ListeningMode mode);
    void AudioLoop();
};

#endif // _APPLICATION_H_
