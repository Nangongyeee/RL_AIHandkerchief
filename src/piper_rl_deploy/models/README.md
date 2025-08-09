# 模型文件放置说明

将你训练好的模型文件放置在此目录下：

## PyTorch 模型 (.pt 文件)
- 将 `.pt` 文件复制到此目录
- 在配置文件中设置 `model_path` 为相对路径，例如：
  ```yaml
  model_path: "models/your_model.pt"
  ```

## ONNX 模型 (.onnx 文件)  
- 将 `.onnx` 文件复制到此目录
- 在配置文件中设置：
  ```yaml
  model_path: "models/your_model.onnx"
  model_type: "onnx"
  ```

## 模型要求

### 输入格式
- 输入维度应与配置文件中的 `obs_dim` 匹配
- 输入数据类型：float32
- 输入形状：[batch_size, obs_dim] 或 [batch_size, history_length * obs_dim]（如果使用历史）

### 输出格式
- 输出维度应与配置文件中的 `action_dim` 匹配
- 输出数据类型：float32
- 输出形状：[batch_size, action_dim]
- 输出值范围：建议在 [-1, 1] 之间

## 示例模型文件
```
models/
├── piper_policy.pt      # PyTorch 模型
├── piper_policy.onnx    # ONNX 模型
└── README.md           # 本说明文件
```

## 使用方法
1. 将模型文件复制到此目录
2. 修改配置文件中的模型路径
3. 启动部署节点：
   ```bash
   ros2 launch piper_rl_deploy piper_rl_deploy.launch.py model_path:=models/your_model.pt
   ```
