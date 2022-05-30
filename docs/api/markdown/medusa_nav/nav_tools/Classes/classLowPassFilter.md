---
title: LowPassFilter

---

# LowPassFilter





## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[LowPassFilter](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-lowpassfilter)**() |
| | **[LowPassFilter](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-lowpassfilter)**(double wn, double qsi) |
| | **[LowPassFilter](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-lowpassfilter)**(double wn, double qsi, bool initialize_on_first) |
| | **[LowPassFilter](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-lowpassfilter)**(double wn, double qsi, double var_init, double var_dot_init) |
| | **[~LowPassFilter](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-~lowpassfilter)**() |
| std::pair< double, double > | **[update](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-update)**(double measurement) |
| std::pair< double, double > | **[predict](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#function-predict)**() |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| bool | **[initialized](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-initialized)**  |

## Protected Attributes

|                | Name           |
| -------------- | -------------- |
| double | **[wn](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-wn)**  |
| double | **[qsi](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-qsi)**  |
| double | **[K1](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-k1)**  |
| double | **[K2](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-k2)**  |
| std::pair< double, double > | **[estimate](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-estimate)**  |
| ros::Time | **[last_Update](/medusa_base/api/markdown/medusa_nav/nav_tools/Classes/classLowPassFilter/#variable-last-update)**  |

## Public Functions Documentation

### function LowPassFilter

```cpp
inline LowPassFilter()
```


### function LowPassFilter

```cpp
inline LowPassFilter(
    double wn,
    double qsi
)
```


### function LowPassFilter

```cpp
inline LowPassFilter(
    double wn,
    double qsi,
    bool initialize_on_first
)
```


### function LowPassFilter

```cpp
inline LowPassFilter(
    double wn,
    double qsi,
    double var_init,
    double var_dot_init
)
```


### function ~LowPassFilter

```cpp
inline ~LowPassFilter()
```


### function update

```cpp
inline std::pair< double, double > update(
    double measurement
)
```


### function predict

```cpp
inline std::pair< double, double > predict()
```


## Public Attributes Documentation

### variable initialized

```cpp
bool initialized;
```


## Protected Attributes Documentation

### variable wn

```cpp
double wn;
```


### variable qsi

```cpp
double qsi;
```


### variable K1

```cpp
double K1;
```


### variable K2

```cpp
double K2;
```


### variable estimate

```cpp
std::pair< double, double > estimate;
```


### variable last_Update

```cpp
ros::Time last_Update;
```


-------------------------------

Updated on 2022-05-30 at 08:04:28 +0000