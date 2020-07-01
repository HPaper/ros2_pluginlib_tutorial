# ROS2에서 Pluginlib 쓰기

Created: Jun 30, 2020 11:39 AM
Property: HYEJONG KIM
Tags: ROS2

---

이 문서는 pluginlib Tutorials를 베이스로 작성 되었으나, ROS2에 맞도록 수정되었다.

[Wiki](http://wiki.ros.org/pluginlib)

[](http://library.isr.ist.utl.pt/docs/roswiki/pluginlib(2f)Tutorials(2f)Writing(20)and(20)Using(20)a(20)Simple(20)Plugin.html)

# 개요

이 문서에서는 ROS2에서 `pluginlib`를 사용하여 **Plugin**을 제작하는 방법에 대해 설명한다. pluginlib는 현재 navigation2, rviz2, rqt등 ROS2 기본 패키지에서 다양하게 사용되고 있다. 

`pluginlib`는 ROS 패키지 내에서 **Plugin**을 로드 및 언로드하기위한 C ++ 라이브러리이다. **Plugin**은 **Runtime library**(예 : shared object, dynamically linked library)에서 로드되는 동적으로 로드 가능한 Class이다. `pluginlib`를 사용하면 Class를 포함하는 Library에 대해 애플리케이션을 명시적으로 링크 할 필요가 없다. 대신 `pluginlib`는 애플리케이션이 라이브러리 또는 클래스 정의를 포함하는 헤더 파일에 대한 사전 인식없이 언제든지 내 보낸 클래스가 포함 된 라이브러리를 열 수 있습니다. **Plugin**은 응용 프로그램 **소스 코드가 없어도 응용 프로그램 동작을** **확장/수정하는 데 유용**하다.

## Plugin의 이점

특정 알고리즘을 포함하는 **Class**를 **Library**형태로 읽어와 쓰지않고, **** `pluginlib`를 이용해 **Plugin**을 만들어 사용하였을때의 이 점은, 

> Application node에서 알고리즘 Class를 가져다 쓸때, Application의 실행 코드를 수정할 필요가 없다.

는 점이다. 이로 인해 알고리즘을 바꿔 테스트하는데 

1. build 시간을 추가로 들일 필요가 없다.
2. User가 알고리즘을 바꾸고 싶을 때, user는 plugin class만 만들어, Application을 실행할 때, arg로 넣어주는 형태로 자신을 알고리즘을 적용할 수 있다.
3. Application을 공개할 때 소스코드를 공개하지 않아도, User가 알고리즘을 바꿀 수 있다.

## 구성 요소

plugin을 구성할 때는 총 3가지 요소가 필요하다.

- Base class (껍데기)
    - 특정 알고리즘을 구현할 plugin class의 형태를 정해주는 virtual functions으로 이루어진 가상 Class (virtual function이 아닌 함수도 물론 추가 할 수 있다.)
- plugin class (알갱이)
    - Base class를 상속 받아 알고리즘을 구현하는 class이다.
- application node
    - 알고리즘을 실제로 실행하는 실행 파일로 plugin을 제작하는 user에게 공개될 필요가 없는 source code이다

3 가지 명칭에 대해서는 본 문서에서 알기 쉽게 정한 명칭으로 통상적으로 쓰이는 명칭은 아닐 수 있다.

# 체크리스트

일반적인 ROS node 혹은 library를 작성하는 때와는 다르게 Plugin을 작성함에 있어, 주의하고 추가해야할 사항만을 정리한다. 

## Base Class

일반적인 추상화 함수를 작성. 특별히 plugin을위해 해야하는 작업 없음.

## Application node

- [ ]  `pluginlib::ClassLoader`를 이용한 plugin class 불러오기 (source code)

    ```cpp
    pluginlib::ClassLoader<base_class_namespace::base_class_name> class_loader_("base_class_namespace", "base_class_namespace::base_class_name");
    std::shared_ptr<base_class_namespace::base_class_name> plugin_class_ = nullptr;
    plugin_class_ = class_loader_.createSharedInstance("plugin_class_namespace/plugin_class_name");
    ```

## Plugin Class

- [ ]  `Base class`를 상속받은 `plugin class` 작성 (source code)
- [ ]  `PLUGINLIB_EXPORT_CLASS`를 이용한 Plugin 등록 (source code)

    ```cpp
    PLUGINLIB_EXPORT_CLASS(plugin_class_namespace::plugin_class_name, base_class_namespace::base_class_name)
    ```

- [ ]  `Plugin Description File` 작성 (plugin_description_file_name.xml)

    ```cpp
    <library path="plugin_class_pkg_name">
      <class name="plugin_class_namespace/plugin_class_name" type="plugin_class_namespace::plugin_class_name" base_class_type="base_class_namespace::base_class_name">
        <description> description.</description>
      </class>
    </library>
    ```

- [ ]  `Plugin Description File` 내보내기 (CMakeLists.txt)

    ```cpp
    install(FILES plugin_description_file_name.xml
      DESTINATION share/${PROJECT_NAME}
    )
    pluginlib_export_plugin_description_file(base_class_pkg_name plugin_description_file_name.xml)
    ```

# 예제

---

## Base Class

### 패키지 생성

ROS2 workspace에 ros2 package를 만든다.

```bash
ros2 pkg create polygon_base
```

### 소스 코드 작성

모든 플러그인이 상속 할 기본 클래스를 만든다. 이 예제에서는 몇 가지 `RegularPolygon` 객체를 만들어 사용할 것이므로 `RegularPolygon` 클래스를 만들어야한다. 

선호하는 편집기를 열고 include/polygon_base/polygon_base.h를 만들어 아래 코드 `polygon_base.h`를 다음을 붙여 넣는다. 

command line에서 바로 파일을 만들어 작성하고 싶다면, 아래 command를 입력해 파일을 작성한다.

```bash
cd polygon_base/include/polygon_base
> polygon_base.h
```

- `polygon_base.h`

    ```cpp
    #ifndef PLUGINLIB_TUTORIALS_POLYGON_BASE_H_
    #define PLUGINLIB_TUTORIALS_POLYGON_BASE_H_

    namespace polygon_base
    {
      class RegularPolygon
      {
        public:
          virtual void initialize(double side_length) = 0;
          virtual double area() = 0;
          virtual ~RegularPolygon(){}

        protected:
          RegularPolygon(){}
      };
    };
    #endif
    ```

위 code에서는 `RegularPolygon`이라는 추상 클래스를 만들었다. 주목해야 할 것은 initialize 함수가 있다는 것이다. `pluginlib`를 사용하면 클래스에 매개 변수가없는 생성자가 필요하므로 매개 변수가 필요한 경우 initialize 함수를 이용해 객체를 초기화해야한다.

### CmakeLists.txt

Base class를 상속받아 사용할 plugin class가 Base class와 다른 패키지에 존재한다면, Base class의 include files를 CMakeLists.txt 파일에서 `install` 해주고, `ament_export_include_directories` 함수를 이용해 외부에서 사용할 수 있도록 export한다.

`polygon_base` 패키지의 `CmakeLists.txt`파일을 열어 아래 코드들을 추가해준다.

```bash
install(
  DIRECTORY include/
  DESTINATION include/
)
```

```bash
ament_export_include_directories(include)
```

- `CmakeLists.txt`

    ```cpp
    cmake_minimum_required(VERSION 3.5)
    project(polygon_base)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)

    install(
      DIRECTORY include/
      DESTINATION include/
    )

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()

    ament_export_include_directories(include)

    ament_package()
    ```

만약 Base class가 virtual function 이외의 실행함수를 가지며, `.cpp` 파일을 가진다면, include을 install 할 뿐아니라, `add_library` 함수를 이용해 라이브러리화 해주어야한다.

### package.xml

Base Class 패키지에 있어서는 package.xml 파일에 pluginlib를 쓰기위해 특별히 추가해줘야하는 사항은 없다.

---

## Application node

### 패키지 생성

이제 RegularPolygon 플러그인을 사용하는 패키지를 만들어 보자. ROS2 workspace에 ros2 package를 만든다. 이때 `plugin_application`패키지는 `rclcpp`, `pluginlib` 와 `polygon_base` 에 의존성을 가진다.

```bash
ros2 pkg create plugin_application --dependencies rclcpp pluginlib polygon_base
```

### 소스코드 작성

src/polygon_loader.cpp를 만들고 다음을 붙여 넣습니다.

```bash
cd plugin_application/src
> polygon_loader.cpp
```

- `polygon_loader.cpp`

    ```cpp
    #include "rclcpp/rclcpp.hpp"
    #include <pluginlib/class_loader.h>
    #include "polygon_base/polygon_base.h"

    namespace polygon_loader
    {
    class PolygonLoader : public rclcpp::Node
    {
    private:
      pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader_;
      std::shared_ptr<polygon_base::RegularPolygon> triangle_;
      std::shared_ptr<polygon_base::RegularPolygon> square_;

    public:
      PolygonLoader()
        : Node("polygon_loader")
        , poly_loader_("polygon_base", "polygon_base::RegularPolygon")
        , triangle_(nullptr)
        , square_(nullptr)
      {
        try
        {
          triangle_ = poly_loader_.createSharedInstance("polygon_plugins/regular_triangle");
          triangle_->initialize(10.0);

          square_ = poly_loader_.createSharedInstance("polygon_plugins/regular_square");
          square_->initialize(10.0);

          RCLCPP_INFO(this->get_logger(),"Triangle area: %.2f", triangle_->area());
          RCLCPP_INFO(this->get_logger(),"Square area: %.2f", square_->area());
        }
        catch(pluginlib::PluginlibException& ex)
        {
          RCLCPP_ERROR(this->get_logger(),"The plugin failed to load for some reason. Error: %s", ex.what());
        }
      }

      ~PolygonLoader(){}
    };
    }

    int main(int argc, char** argv)
    {
      setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

      rclcpp::init(argc, argv);

      rclcpp::executors::SingleThreadedExecutor executor;

      auto polygon_loader = std::make_shared<polygon_loader::PolygonLoader>();

      executor.add_node(polygon_loader);
      executor.spin();

      rclcpp::shutdown();

      return 0;
    }
    ```

여기에서 중요한 몇가지 코드를 확인해보자.

```cpp
#include <pluginlib/class_loader.h>
#include "polygon_base/polygon_base.h"
```

`pluginlib`를 통해 plugin class를 불러오기 위해서는 `pluginlib`의 `ClassLoader`와 미리 만들어둔 plugin base인 `RegularPolygon`를 include해야한다.

```cpp
pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader_;
```

여기에서는 플러그인을로드하는 데 사용할 `ClassLoader`를 만든다. Base class의 정규화 된 유형을 template으로 넣는다.(이 경우 polygon_base :: RegularPolygon).

```cpp
  std::shared_ptr<polygon_base::RegularPolygon> triangle_;
  std::shared_ptr<polygon_base::RegularPolygon> square_;
```

여기서는 plugin 객체를 load하여 사용할 객체 포인터를 선언한다. 추후 shared_ptr로 객체를 불러올 것이기 때문에 `std::shared_ptr`의 형태로 만든다.

```cpp
PolygonLoader()
    : Node("polygon_loader")
    , poly_loader_("polygon_base", "polygon_base::RegularPolygon")
    , triangle_(nullptr)
    , square_(nullptr)
  {
```

ros2 node class의 생성자에서 `pluginlib::ClassLoader`를 초기화 할때, 두가지 인자가 필요하다. 첫 번째는 플러그인 기본 클래스를 포함하는 패키지 이름(이 경우 polygon_base), 두 번째는 Base class의 정규화 된 유형(이 경우 polygon_base :: RegularPolygon)이다 .

```cpp
triangle_ = poly_loader_.createSharedInstance("polygon_plugins/regular_triangle");
triangle_->initialize(10.0);
```

여기서는 실제로 `polygon_plugins/regulare_triangle` 플러그인을 로드한다. 플러그인 생성자는 인수를 가질 수 없으므로 객체를 초기화하도록 객체를 생성 한 직후에 initialize 함수를 호출한다.

### CMakeLists.txt

위 소스코드를 빌드하기위해서는 CMakeLists.txt, package.xml 파일에서 Dependency 설정을 해주어야한다. 

패키지를 생성할때 Dependency를 잘 설정해서 만들었다면, 일부 코드는 이미 작성되어 있을것이다.

`plugin_application` 패키지의 `CmakeLists.txt`파일을 열어 아래 코드들을 추가해준다.

```bash
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)
```

`pluginlib`와 `polygon_base` 패키지를 찾는다.

```bash
add_executable(polygon_loader
    src/polygon_loader.cpp)
ament_target_dependencies(polygon_loader
  "rclcpp"
  "pluginlib"
  "polygon_base"
  )
install(TARGETS polygon_loader
    DESTINATION lib/${PROJECT_NAME}
    )
```

`polygon_loader` 노드를 `add_executable`로 빌드할때 `ament_target_dependencies` 함수로 의존성을 설정해준다.

- `CmakeLists.txt`

    ```cpp
    cmake_minimum_required(VERSION 3.5)
    project(plugin_application)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(polygon_base REQUIRED)

    add_executable(polygon_loader
        src/polygon_loader.cpp)
    ament_target_dependencies(polygon_loader
      "rclcpp"
      "pluginlib"
      "polygon_base"
      )
    install(TARGETS polygon_loader
        DESTINATION lib/${PROJECT_NAME}
        )

    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
    ```

### package.xml

`plugin_application` 패키지의 `package.xml` 파일을 열어 아래 코드들을 추가해준다.

```cpp
<depend>rclcpp</depend>
<depend>pluginlib</depend>
<depend>polygon_base</depend>
```

---

## plugin class

### 패키지 생성

Plugin 패키지를 만들어 보자. ROS2 workspace에 ros2 package를 만든다. 이때 `polygon_plugins` 패키지는 `pluginlib` 와 `polygon_base` 에 의존성을 가진다.

```bash
ros2 pkg create polygon_plugins --dependencies pluginlib polygon_base
```

### Source code 작성

polygon_plugins/include/polygon_plugins/polygon_plugins.h를 만들고 다음을 붙여 넣습니다.

```bash
cd polygon_plugins/include/polygon_plugins/
> polygon_plugins.h
```

- `polygon_plugins.h`

    ```cpp
    #ifndef PLUGINLIB_TUTORIALS_POLYGON_PLUGINS_H_
    #define PLUGINLIB_TUTORIALS_POLYGON_PLUGINS_H_

    #include <cmath>
    #include <polygon_base/polygon_base.h>
    #include <pluginlib/class_list_macros.h>

    namespace polygon_plugins
    {
      class Triangle : public polygon_base::RegularPolygon
      {
        public:
          Triangle(){}
          void initialize(double side_length);
          double area();
          double getHeight();

        private:
          double side_length_;
      };

      class Square : public polygon_base::RegularPolygon
      {
        public:
          Square(){}
          void initialize(double side_length);
          double area();

        private:
          double side_length_;
      };
    }
    #endif
    ```

여기에서 플러그인으로 사용할 `RegularPolygon`를 상속받는 두 개의 클래스를 만들었다.

또한 이 파일에는 플러그인 Class를 선언하는데 사용하기 위해 다음의 헤더파일이 선언되어있다.

```cpp
#include <pluginlib/class_list_macros.h>
```

polygon_loader/src/polygon_loader.cpp를 만들고 다음을 붙여 넣습니다.

```cpp
cd polygon_plugins/src
> polygon_plugins.cpp
```

- `polygon_plugins.cpp`

    ```cpp
    #include "polygon_plugins/polygon_plugins.h"

    void polygon_plugins::Triangle::initialize(double side_length)
    {
      side_length_ = side_length;
    }

    double polygon_plugins::Triangle::area()
    {
      return 0.5 * side_length_ * getHeight();
    }

    double polygon_plugins::Triangle::getHeight()
    {
      return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
    }

    void polygon_plugins::Square::initialize(double side_length)
    {
      side_length_ = side_length;
    }

    double polygon_plugins::Square::area()
    {
      return side_length_ * side_length_;
    }

    PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
    PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
    ```

cpp 파일에는 일반적인 c++ 클래스 `Triangle` 및 `Square` 가 선언되어 있고, 그 아래 두 클래스를 플러그인으로 선언하기위해 다음을 붙여 넣었다.

```cpp
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
```

여기서는 `Triangle`와 `Square` 클래스를 플러그인으로 등록했다. 클래스의 플러그인 등록은  `PLUGINLIB_EXPORT_CLASS` 매크로로 이루어지며, 이 매크로의 인자는 아래 2가지이다.

1. 플러그인 클래스의 자료형 (이 경우 `polygon_plugins::Triangle`, `polygon_plugins::Square`) 
2. 기본 클래스의 정규화된 자료형 (이 경우 `polygon_base::RegularPolygon`)

### Plugin Description File 작성

위의 단계를 통해 플러그인 라이브러리가 로드되면 플러그인 인스턴스를 만들 수 있지만, `plugin loader`가 해당 라이브러리의 path를 찾고, 해당 라이브러리 내에서 참조해야 할 것을 알 수 있도록 등록해야한다. 이를 위해 추후에 설명할 CMakelists.txt의 `pluginlib_export_plugin_description_file`과 함께, 플러그인에 필요한 모든 정보를 ROS 툴체인에서 사용할 수 있도록하는 XML 파일도 작성한다.

polygon_loader패키지 폴더에 `Plugin Description File`(.xml) 파일을 만들고, 다음을 붙여 넣는다. 이 `Plugin Description File`파일은 (CMakeLists.txt 및 package.xml과 함께). 패키지의 최상위 레벨에 있어야합니다. (이 예시에서는 이름을 `plugin.xml` 로 정했다.

```cpp
cd polygon_plugins
> plugin.xml
```

- `plugin.xml`

    ```cpp
    <library path="polygon_plugins">
      <class name="polygon_plugins/regular_triangle" type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
        <description>This is a triangle plugin.</description>
      </class>
      <class name="polygon_plugins/regular_square" type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
        <description>This is a square plugin.</description>
      </class>
    </library>
    ```

우리가 더 자세히 살펴볼 두 줄이 있다.

```cpp
<library path="polygon_plugins">
```

library태그는 내보내려는 플러그인이 포함 된 라이브러리의 상대 경로를 제공합니다. 이 경우에는 `polygon_plugins`이고, 일반적으로 패키지 명을 따른다.

```cpp
<class name="polygon_plugins/regular_triangle" type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
  <description>This is a triangle plugin.</description>
</class>
```

class 태그는 라이브러리에서 내보내려는 플러그인을 선언한다. 그 매개 변수로는 아래와 같은 것들이 있다.

- `name` : 일반적으로 `plugin_namespace/PluginClassName`으로 지정된 플러그인의 이름. 패키지 이름을 `regular_triangle`, 네임 스페이스를 `polygon_plugins`로 사용했기 때문에 `polygon_plugins/regular_triangle`을 사용합니다.
- `type` : 플러그인의 정규화 된 유형. 예시에서는 `polygon_plugins :: Triangle`이다.
- `base_class` : 플러그인의 완전한 기본 클래스 유형. 예시에서는 `polygon_base :: RegularPolygon`이다.
- `description` : 플러그인에 대한 설명 및 기능.

### CMakeLists.txt

위 소스코드를 빌드하기위해서는 CMakeLists.txt, package.xml 파일에서 Dependency 설정을 해주어야한다. 

패키지를 생성할때 Dependency를 잘 설정해서 만들었다면, 일부 코드는 이미 작성되어 있을것이다.

`polygon_plugins` 패키지의 `CmakeLists.txt`파일을 열어 아래 코드들을 추가해준다.

```bash
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)
```

`pluginlib`와 `polygon_base` 패키지를 찾는다.

```cpp
include_directories(include)
```

source code에서 현재 패키지의 include file을 읽어 올수 있도록 추가한다.

```bash
add_library(polygon_plugins SHARED
    src/polygon_plugins.cpp)
target_compile_definitions(polygon_plugins
PRIVATE "MANIPULATOR_IK_SOLVOR_PLUGINS_BUILDING_DLL"
#    PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)
ament_target_dependencies(polygon_plugins
  "pluginlib"
  "polygon_base"
)
```

소스코드를 `polygon_plugins` 라이브러리로 빌드한다. 이때 `ament_target_dependencies` 함수로 의존성을 설정해준다. 

```cpp
install(TARGETS polygon_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(polygon_plugins)
```

`polygon_plugins` 라이브러리와 `include` 파일을 install하고, export한다.

여기까지는 일반적인 ROS2 Library를 컴파일하기 위한 옵션이다. 여기에 추가적으로 `CMakelists.txt`에 `pluginlib_export_plugin_description_file`을 선언하여, `Plugin Description File`에 작성된 모든 정보를 ROS 툴체인에서 사용할 수 있도록 install, export해주어야한다.

```cpp
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(polygon_base plugin.xml)
```

`Plugin Description File`을 install하고, export한다. 이때 `pluginlib_export_plugin_description_file`의 첫번째 인자값은 플러그인의 base_class가있는 패키지와 일치해야한다. 두번째 인자는 위 단계에서 생성 된 XML 파일을 가리 키도록 설정한다.

- `CmakeLists.txt`

### package.xml

`plugin_application` 패키지의 `package.xml` 파일을 열어 아래 코드들을 추가해준다.

```cpp
<depend>rclcpp</depend>
<depend>pluginlib</depend>
<depend>polygon_base</depend>
```