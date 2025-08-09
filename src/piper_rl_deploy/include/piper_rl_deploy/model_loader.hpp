#pragma once

#include <memory>
#include <vector>
#include <string>
#include <deque>
#include <mutex>

#ifdef USE_PYTORCH
#include <torch/torch.h>
#include <torch/script.h>
#endif

#ifdef USE_ONNX
#include <onnxruntime_cxx_api.h>
#endif

namespace piper_rl_deploy {

enum class ModelType {
    PYTORCH,
    ONNX,
    NONE
};

class ModelLoader {
public:
    explicit ModelLoader(const std::string& model_path, ModelType type = ModelType::PYTORCH);
    ~ModelLoader() = default;

    bool loadModel();
    bool isModelLoaded() const { return model_loaded_; }
    
    std::vector<float> inference(const std::vector<float>& input);
    
    void setInputDim(size_t dim) { input_dim_ = dim; }
    void setOutputDim(size_t dim) { output_dim_ = dim; }
    
    size_t getInputDim() const { return input_dim_; }
    size_t getOutputDim() const { return output_dim_; }

private:
    std::string model_path_;
    ModelType model_type_;
    bool model_loaded_;
    
    size_t input_dim_;
    size_t output_dim_;
    
#ifdef USE_PYTORCH
    torch::jit::script::Module torch_model_;
#endif

#ifdef USE_ONNX
    std::unique_ptr<Ort::Session> onnx_session_;
    std::unique_ptr<Ort::Env> onnx_env_;
    Ort::SessionOptions session_options_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
#endif

    std::vector<float> inferenceWithPyTorch(const std::vector<float>& input);
    std::vector<float> inferenceWithONNX(const std::vector<float>& input);
};

// 观测历史缓存
template<typename T>
class HistoryBuffer {
public:
    explicit HistoryBuffer(size_t max_size) : max_size_(max_size) {}
    
    void push(const T& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.push_back(data);
        if (buffer_.size() > max_size_) {
            buffer_.pop_front();
        }
    }
    
    std::vector<T> getBuffer() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::vector<T>(buffer_.begin(), buffer_.end());
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }
    
private:
    size_t max_size_;
    std::deque<T> buffer_;
    mutable std::mutex mutex_;
};

} // namespace piper_rl_deploy
