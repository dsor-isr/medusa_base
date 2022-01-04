# Medusa Gimmicks Library

## Description

Collection of methods to be used by several nodes in the Medusa stack.

---

## Important Sources

The library was built following the steps in this [link](https://roboticsbackend.com/ros-include-cpp-header-from-another-package/)
Also an important note about implementing template methods [here](https://stackoverflow.com/questions/1353973/c-template-linking-error). Basically this avoids linking errors with the library.

---

## How to use the library in a new package/node

#### CmakeLists.txt
Add library to find package 
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  medusa_gimmicks_lirabry
)
```

#### package.xml
Add the following
```
<depend>medusa_gimmicks_lirabry</depend>
```

#### Include the library in your new node

```
#include <medusa_gimmicks_library/MedusaGimmicks.h>
```

#### Use in your code

```
MedusaGimmicks::method_to_use(...)
```
---

## Package Content

![medusa_gimmicks_library struct](img/medusa_gimmicks_library_structure.png)

## Code documentation

[source](http://lungfish.isr.tecnico.ulisboa.pt/medusa_vx_doxy/medusa_addons/medusa_gimmicks_library/html/index.html)

