#!/usr/bin/env python3
"""
NeuPAN Nav2 控制器插件注册测试脚本
用于验证插件是否正确注册到 Nav2 系统中

作者: NeuPAN Team
许可证: GNU General Public License v3.0
"""

import subprocess
import sys
import time
import os
import xml.etree.ElementTree as ET
from pathlib import Path


class PluginRegistrationTester:
    """插件注册测试器"""
    
    def __init__(self):
        self.package_name = "neupan_nav2_controller"
        self.plugin_class = "neupan_nav2_controller::NeuPANController"
        self.base_class = "nav2_core::Controller"
        self.plugin_xml = "neupan_controller_plugin.xml"
        
        self.test_results = {
            "package_found": False,
            "plugin_xml_found": False,
            "plugin_xml_valid": False,
            "plugin_registered": False,
            "library_found": False,
            "dependencies_ok": False
        }
        
    def run_all_tests(self):
        """运行所有测试"""
        print("🔍 开始 NeuPAN Nav2 控制器插件注册测试")
        print("=" * 60)
        
        self.test_package_exists()
        self.test_plugin_xml_exists()
        self.test_plugin_xml_valid()
        self.test_plugin_registration()
        self.test_library_exists()
        self.test_dependencies()
        
        self.print_summary()
        
        return all(self.test_results.values())
    
    def test_package_exists(self):
        """测试包是否存在"""
        print("📦 测试包存在性...")
        
        try:
            result = subprocess.run(
                ["ros2", "pkg", "prefix", self.package_name],
                capture_output=True, text=True, timeout=10
            )
            
            if result.returncode == 0:
                package_path = result.stdout.strip()
                print(f"  ✅ 包找到: {package_path}")
                self.test_results["package_found"] = True
            else:
                print(f"  ❌ 包未找到: {result.stderr}")
                self.test_results["package_found"] = False
                
        except subprocess.TimeoutExpired:
            print("  ❌ 命令超时")
            self.test_results["package_found"] = False
        except Exception as e:
            print(f"  ❌ 错误: {e}")
            self.test_results["package_found"] = False
            
    def test_plugin_xml_exists(self):
        """测试插件XML文件是否存在"""
        print("📄 测试插件XML文件...")
        
        try:
            # 获取包路径
            result = subprocess.run(
                ["ros2", "pkg", "prefix", self.package_name],
                capture_output=True, text=True
            )
            
            if result.returncode == 0:
                package_path = Path(result.stdout.strip())
                xml_path = package_path / "share" / self.package_name / self.plugin_xml
                
                if xml_path.exists():
                    print(f"  ✅ XML文件找到: {xml_path}")
                    self.test_results["plugin_xml_found"] = True
                    self.xml_path = xml_path
                else:
                    print(f"  ❌ XML文件未找到: {xml_path}")
                    self.test_results["plugin_xml_found"] = False
            else:
                print("  ❌ 无法获取包路径")
                self.test_results["plugin_xml_found"] = False
                
        except Exception as e:
            print(f"  ❌ 错误: {e}")
            self.test_results["plugin_xml_found"] = False
            
    def test_plugin_xml_valid(self):
        """测试插件XML文件的有效性"""
        print("🔍 验证插件XML内容...")
        
        if not self.test_results["plugin_xml_found"]:
            print("  ❌ XML文件不存在，跳过验证")
            self.test_results["plugin_xml_valid"] = False
            return
            
        try:
            tree = ET.parse(self.xml_path)
            root = tree.getroot()
            
            # 检查根元素
            if root.tag != "library":
                print(f"  ❌ 根元素应该是 'library'，实际是 '{root.tag}'")
                self.test_results["plugin_xml_valid"] = False
                return
                
            # 检查 path 属性
            if "path" not in root.attrib:
                print("  ❌ 缺少 'path' 属性")
                self.test_results["plugin_xml_valid"] = False
                return
                
            print(f"  📚 库路径: {root.attrib['path']}")
            
            # 查找插件类
            found_plugin = False
            for class_elem in root.findall("class"):
                if class_elem.attrib.get("name") == self.plugin_class:
                    found_plugin = True
                    base_class = class_elem.attrib.get("type")
                    
                    print(f"  🔌 插件类: {self.plugin_class}")
                    print(f"  📏 基类: {base_class}")
                    
                    if base_class == self.base_class:
                        print("  ✅ 基类正确")
                    else:
                        print(f"  ⚠️  基类不匹配，期望 {self.base_class}")
                        
                    # 检查描述
                    desc_elem = class_elem.find("description")
                    if desc_elem is not None:
                        print(f"  📝 描述: {desc_elem.text}")
                    
                    break
                    
            if found_plugin:
                print("  ✅ 插件XML有效")
                self.test_results["plugin_xml_valid"] = True
            else:
                print(f"  ❌ 未找到插件类 {self.plugin_class}")
                self.test_results["plugin_xml_valid"] = False
                
        except ET.ParseError as e:
            print(f"  ❌ XML解析错误: {e}")
            self.test_results["plugin_xml_valid"] = False
        except Exception as e:
            print(f"  ❌ 错误: {e}")
            self.test_results["plugin_xml_valid"] = False
            
    def test_plugin_registration(self):
        """测试插件是否在系统中注册"""
        print("🔌 测试插件注册...")
        
        try:
            # 使用 ros2 plugin list 命令
            result = subprocess.run(
                ["ros2", "plugin", "list", "--packages", self.package_name],
                capture_output=True, text=True, timeout=15
            )
            
            if result.returncode == 0:
                plugins = result.stdout
                if self.plugin_class in plugins:
                    print(f"  ✅ 插件已注册: {self.plugin_class}")
                    self.test_results["plugin_registered"] = True
                else:
                    print("  ❌ 插件未注册")
                    print(f"  📄 可用插件:\n{plugins}")
                    self.test_results["plugin_registered"] = False
            else:
                print(f"  ❌ 命令失败: {result.stderr}")
                # 尝试备用方法
                self.test_plugin_registration_alternative()
                
        except subprocess.TimeoutExpired:
            print("  ❌ 命令超时")
            self.test_results["plugin_registered"] = False
        except Exception as e:
            print(f"  ❌ 错误: {e}")
            self.test_results["plugin_registered"] = False
            
    def test_plugin_registration_alternative(self):
        """备用插件注册测试方法"""
        print("  🔄 尝试备用检测方法...")
        
        try:
            # 检查 pluginlib 是否能找到插件
            result = subprocess.run([
                "ros2", "run", "pluginlib", "pluginlib_headers_check", 
                self.package_name, self.base_class
            ], capture_output=True, text=True, timeout=10)
            
            if self.plugin_class in result.stdout:
                print("  ✅ 备用方法确认插件已注册")
                self.test_results["plugin_registered"] = True
            else:
                print("  ❌ 备用方法未能确认插件注册")
                
        except Exception as e:
            print(f"  ⚠️  备用方法失败: {e}")
            
    def test_library_exists(self):
        """测试共享库是否存在"""
        print("📚 测试共享库文件...")
        
        try:
            result = subprocess.run(
                ["ros2", "pkg", "prefix", self.package_name],
                capture_output=True, text=True
            )
            
            if result.returncode == 0:
                package_path = Path(result.stdout.strip())
                
                # 检查可能的库文件位置
                lib_paths = [
                    package_path / "lib" / f"lib{self.package_name}.so",
                    package_path / "lib" / self.package_name / f"lib{self.package_name}.so",
                ]
                
                found_lib = False
                for lib_path in lib_paths:
                    if lib_path.exists():
                        print(f"  ✅ 共享库找到: {lib_path}")
                        print(f"  📏 文件大小: {lib_path.stat().st_size} bytes")
                        self.test_results["library_found"] = True
                        found_lib = True
                        break
                        
                if not found_lib:
                    print("  ❌ 共享库文件未找到")
                    print("  🔍 搜索的路径:")
                    for lib_path in lib_paths:
                        print(f"    - {lib_path}")
                    self.test_results["library_found"] = False
                    
        except Exception as e:
            print(f"  ❌ 错误: {e}")
            self.test_results["library_found"] = False
            
    def test_dependencies(self):
        """测试依赖项"""
        print("🔗 测试依赖项...")
        
        required_packages = [
            "nav2_core",
            "nav2_common", 
            "pluginlib",
            "geometry_msgs",
            "nav_msgs",
            "sensor_msgs"
        ]
        
        missing_deps = []
        
        for pkg in required_packages:
            try:
                result = subprocess.run(
                    ["ros2", "pkg", "prefix", pkg],
                    capture_output=True, text=True, timeout=5
                )
                
                if result.returncode == 0:
                    print(f"  ✅ {pkg}: 已安装")
                else:
                    print(f"  ❌ {pkg}: 未找到")
                    missing_deps.append(pkg)
                    
            except Exception as e:
                print(f"  ❌ {pkg}: 检查失败 - {e}")
                missing_deps.append(pkg)
                
        if not missing_deps:
            print("  ✅ 所有依赖项满足")
            self.test_results["dependencies_ok"] = True
        else:
            print(f"  ❌ 缺少依赖: {', '.join(missing_deps)}")
            self.test_results["dependencies_ok"] = False
            
    def print_summary(self):
        """打印测试总结"""
        print("\n" + "=" * 60)
        print("📊 测试结果总结")
        print("=" * 60)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        print(f"🎯 总计测试: {total_tests}")
        print(f"✅ 通过测试: {passed_tests}")
        print(f"❌ 失败测试: {total_tests - passed_tests}")
        print(f"📈 成功率: {passed_tests/total_tests*100:.1f}%")
        
        print("\n📋 详细结果:")
        for test_name, result in self.test_results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  - {test_name}: {status}")
            
        if all(self.test_results.values()):
            print("\n🎉 所有测试通过！NeuPAN 控制器插件已正确安装和注册。")
            return True
        else:
            print("\n⚠️  部分测试失败，请检查安装和配置。")
            print("\n🔧 建议的修复步骤:")
            
            if not self.test_results["package_found"]:
                print("  1. 确保包已正确构建: colcon build --packages-select neupan_nav2_controller")
                print("  2. 确保工作空间已正确source: source install/setup.bash")
                
            if not self.test_results["plugin_xml_found"]:
                print("  3. 确保 CMakeLists.txt 中包含 plugin XML 安装指令")
                
            if not self.test_results["library_found"]:
                print("  4. 检查 C++ 编译是否成功")
                print("  5. 确保所有依赖项已正确链接")
                
            if not self.test_results["dependencies_ok"]:
                print("  6. 安装缺失的依赖项: sudo apt install ros-humble-nav2-*")
                
            return False


def main():
    """主函数"""
    print("🚀 NeuPAN Nav2 控制器插件注册测试")
    print(f"🕒 测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    
    tester = PluginRegistrationTester()
    success = tester.run_all_tests()
    
    if success:
        print("\n🎊 恭喜！插件测试全部通过！")
        sys.exit(0)
    else:
        print("\n❌ 插件测试失败，请检查上述建议。")
        sys.exit(1)


if __name__ == "__main__":
    main()