#include "piper_rl_deploy/model_loader.hpp"
#include <iostream>
#include <stdexcept>

namespace piper_rl_deploy {

ModelLoader::ModelLoader(const std::string& model_path, ModelType type)
    : model_path_(model_path)
    , model_type_(type)
    , model_loaded_(false)
    , input_dim_(0)
    , output_dim_(0)
{
#ifdef USE_ONNX
    if (model_type_ == ModelType::ONNX) {
        onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PiperRL");
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    }
#endif
}

bool ModelLoader::loadModel() {
    try {
        if (model_type_ == ModelType::PYTORCH) {
#ifdef USE_PYTORCH
            torch_model_ = torch::jit::load(model_path_);
            torch_model_.eval();
            model_loaded_ = true;
            std::cout << "PyTorch model loaded successfully: " << model_path_ << std::endl;
#else
            std::cerr << "PyTorch support not compiled!" << std::endl;
            return false;
#endif
        } else if (model_type_ == ModelType::ONNX) {
#ifdef USE_ONNX
            onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, model_path_.c_str(), session_options_);
            
            // 获取输入输出信息
            Ort::AllocatorWithDefaultOptions allocator;
            size_t num_input_nodes = onnx_session_->GetInputCount();
            size_t num_output_nodes = onnx_session_->GetOutputCount();
            
            input_names_.reserve(num_input_nodes);
            output_names_.reserve(num_output_nodes);
            
            for (size_t i = 0; i < num_input_nodes; i++) {
                char* input_name = onnx_session_->GetInputName(i, allocator);
                input_names_.push_back(input_name);
                
                Ort::TypeInfo input_type_info = onnx_session_->GetInputTypeInfo(i);
                auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
                std::vector<int64_t> input_dims = input_tensor_info.GetShape();
                
                if (i == 0 && input_dims.size() >= 2) {
                    input_dim_ = input_dims[1]; // 假设 batch_size 在第0维
                }
            }
            
            for (size_t i = 0; i < num_output_nodes; i++) {
                char* output_name = onnx_session_->GetOutputName(i, allocator);
                output_names_.push_back(output_name);
                
                Ort::TypeInfo output_type_info = onnx_session_->GetOutputTypeInfo(i);
                auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
                std::vector<int64_t> output_dims = output_tensor_info.GetShape();
                
                if (i == 0 && output_dims.size() >= 2) {
                    output_dim_ = output_dims[1]; // 假设 batch_size 在第0维
                }
            }
            
            model_loaded_ = true;
            std::cout << "ONNX model loaded successfully: " << model_path_ << std::endl;
            std::cout << "Input dim: " << input_dim_ << ", Output dim: " << output_dim_ << std::endl;
#else
            std::cerr << "ONNX support not compiled!" << std::endl;
            return false;
#endif
        } else {
            std::cerr << "Unsupported model type!" << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        return false;
    }
    
    return model_loaded_;
}

std::vector<float> ModelLoader::inference(const std::vector<float>& input) {
    if (!model_loaded_) {
        std::cerr << "Model not loaded!" << std::endl;
        return {};
    }
    
    if (model_type_ == ModelType::PYTORCH) {
        return inferenceWithPyTorch(input);
    } else if (model_type_ == ModelType::ONNX) {
        return inferenceWithONNX(input);
    }
    
    return {};
}

std::vector<float> ModelLoader::inferenceWithPyTorch(const std::vector<float>& input) {
#ifdef USE_PYTORCH
    try {
        // 转换输入为 torch tensor
        torch::Tensor input_tensor = torch::from_blob(
            const_cast<float*>(input.data()), 
            {1, static_cast<long>(input.size())}, 
            torch::kFloat
        ).clone();
        
        // 推理
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);
        
        torch::jit::IValue output = torch_model_.forward(inputs);
        torch::Tensor output_tensor = output.toTensor();
        
        // 转换输出为 vector
        output_tensor = output_tensor.cpu();
        std::vector<float> result(output_tensor.data_ptr<float>(), 
                                 output_tensor.data_ptr<float>() + output_tensor.numel());
        
        return result;
    } catch (const std::exception& e) {
        std::cerr << "PyTorch inference error: " << e.what() << std::endl;
        return {};
    }
#else
    (void)input; // 避免未使用变量警告
    std::cerr << "PyTorch support not compiled!" << std::endl;
    return {};
#endif
}

std::vector<float> ModelLoader::inferenceWithONNX(const std::vector<float>& input) {
#ifdef USE_ONNX
    try {
        // 创建输入 tensor
        std::vector<int64_t> input_shape = {1, static_cast<int64_t>(input.size())};
        
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, 
            const_cast<float*>(input.data()), 
            input.size(), 
            input_shape.data(), 
            input_shape.size()
        );
        
        // 运行推理
        auto output_tensors = onnx_session_->Run(
            Ort::RunOptions{nullptr}, 
            input_names_.data(), 
            &input_tensor, 
            1, 
            output_names_.data(), 
            1
        );
        
        // 提取输出
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
        
        return std::vector<float>(output_data, output_data + output_size);
        
    } catch (const std::exception& e) {
        std::cerr << "ONNX inference error: " << e.what() << std::endl;
        return {};
    }
#else
    (void)input; // 避免未使用变量警告
    std::cerr << "ONNX support not compiled!" << std::endl;
    return {};
#endif
}

} // namespace piper_rl_deploy
