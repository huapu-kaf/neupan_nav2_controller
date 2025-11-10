#include <iostream>
#include <Python.h>
#include <dlfcn.h>
#include <string>
#include <vector>

void test_python_init() {
    std::cout << "=== NeuPAN Python初始化测试 ===" << std::endl;
    
    // 1. 测试Python库预加载
    std::cout << "\n1. 预加载Python库..." << std::endl;
    void* python_lib = dlopen("/usr/lib/x86_64-linux-gnu/libpython3.10.so.1.0", RTLD_LAZY | RTLD_GLOBAL);
    if (python_lib) {
        std::cout << "✅ Python库预加载成功" << std::endl;
    } else {
        std::cout << "❌ Python库预加载失败: " << dlerror() << std::endl;
    }
    
    // 2. 初始化Python解释器
    std::cout << "\n2. 初始化Python解释器..." << std::endl;
    if (!Py_IsInitialized()) {
        Py_Initialize();
        if (Py_IsInitialized()) {
            std::cout << "✅ Python解释器初始化成功" << std::endl;
        } else {
            std::cout << "❌ Python解释器初始化失败" << std::endl;
            return;
        }
    } else {
        std::cout << "✅ Python解释器已经初始化" << std::endl;
    }
    
    // 3. 测试基础模块
    std::cout << "\n3. 测试基础Python模块..." << std::endl;
    std::vector<std::string> basic_modules = {"sys", "os", "math", "numpy"};
    
    for (const auto& module_name : basic_modules) {
        std::string import_cmd = "import " + module_name;
        if (PyRun_SimpleString(import_cmd.c_str()) == 0) {
            std::cout << "✅ 成功导入模块: " << module_name << std::endl;
        } else {
            std::cout << "❌ 导入模块失败: " << module_name << std::endl;
            PyErr_Print();
        }
    }
    
    // 4. 测试Python路径设置
    std::cout << "\n4. 设置Python路径..." << std::endl;
    std::vector<std::string> search_paths = {
        "import sys",
        "import os",
        "sys.path.append('/home/robotmaster/ros2_ws/src/NeuPAN')",
        "sys.path.append('/home/robotmaster/ros2_ws/src/NeuPAN-main')",
        "sys.path.append(os.path.expanduser('~/ros2_ws/src/NeuPAN'))",
        "sys.path.append(os.path.expanduser('~/ros2_ws/src/NeuPAN-main'))"
    };
    
    for (const auto& path_cmd : search_paths) {
        if (PyRun_SimpleString(path_cmd.c_str()) == 0) {
            std::cout << "✅ 路径设置成功: " << path_cmd.substr(0, 30) << "..." << std::endl;
        } else {
            std::cout << "❌ 路径设置失败: " << path_cmd.substr(0, 30) << "..." << std::endl;
        }
    }
    
    // 5. 测试NeuPAN模块导入
    std::cout << "\n5. 测试NeuPAN模块导入..." << std::endl;
    
    // 首先检查可用的模块
    if (PyRun_SimpleString("print('Python路径:', sys.path[-5:])") != 0) {
        std::cout << "❌ 无法打印Python路径" << std::endl;
    }
    
    // 尝试导入neupan模块
    PyObject* neupan_module = PyImport_ImportModule("neupan.neupan");
    if (neupan_module) {
        std::cout << "✅ 成功导入neupan.neupan模块" << std::endl;
        
        // 检查neupan类
        PyObject* neupan_class = PyObject_GetAttrString(neupan_module, "neupan");
        if (neupan_class) {
            std::cout << "✅ 成功获取neupan类" << std::endl;
            
            // 检查init_from_yaml方法
            PyObject* init_func = PyObject_GetAttrString(neupan_class, "init_from_yaml");
            if (init_func && PyCallable_Check(init_func)) {
                std::cout << "✅ 成功获取init_from_yaml方法" << std::endl;
                Py_DECREF(init_func);
            } else {
                std::cout << "❌ 无法获取init_from_yaml方法" << std::endl;
            }
            Py_DECREF(neupan_class);
        } else {
            std::cout << "❌ 无法获取neupan类" << std::endl;
            PyErr_Print();
        }
        Py_DECREF(neupan_module);
    } else {
        std::cout << "❌ 无法导入neupan.neupan模块" << std::endl;
        PyErr_Print();
        
        // 尝试其他可能的导入路径
        std::cout << "\n尝试其他导入方式..." << std::endl;
        std::vector<std::string> alt_imports = {
            "neupan",
            "NeuPAN.neupan.neupan", 
            "NeuPAN.neupan"
        };
        
        for (const auto& import_name : alt_imports) {
            PyObject* module = PyImport_ImportModule(import_name.c_str());
            if (module) {
                std::cout << "✅ 成功导入: " << import_name << std::endl;
                Py_DECREF(module);
                break;
            } else {
                std::cout << "❌ 无法导入: " << import_name << std::endl;
                PyErr_Clear(); // 清除错误以便继续测试
            }
        }
    }
    
    // 6. 测试配置文件路径
    std::cout << "\n6. 检查配置文件..." << std::endl;
    std::vector<std::string> config_paths = {
        "/home/robotmaster/ros2_ws/src/neupan_ros/example/gazebo_limo/config/neupan_planner_limo.yaml",
        "/home/robotmaster/ros2_ws/src/neupan_ros/example/gazebo_limo/pretrain_limo/model_5000.pth"
    };
    
    for (const auto& path : config_paths) {
        std::string check_cmd = "import os; print('文件存在:', os.path.exists('" + path + "'), '" + path + "')";
        if (PyRun_SimpleString(check_cmd.c_str()) == 0) {
            std::cout << "✅ 检查配置文件: " << path.substr(path.find_last_of('/') + 1) << std::endl;
        } else {
            std::cout << "❌ 无法检查配置文件: " << path.substr(path.find_last_of('/') + 1) << std::endl;
        }
    }
    
    std::cout << "\n=== Python初始化测试完成 ===" << std::endl;
}

int main() {
    test_python_init();
    
    // 清理
    if (Py_IsInitialized()) {
        Py_Finalize();
        std::cout << "\n🧹 Python解释器已清理" << std::endl;
    }
    
    return 0;
}