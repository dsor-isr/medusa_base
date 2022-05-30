---
title: CircularDelay::reverse_iterator

---

# CircularDelay::reverse_iterator





## Public Types

|                | Name           |
| -------------- | -------------- |
| typedef [reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#typedef-self-type)**  |
| typedef std::bidirectional_iterator_tag | **[iterator_category](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#typedef-iterator-category)**  |
| typedef int | **[difference_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#typedef-difference-type)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-reverse-iterator)**(const [CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/)< type, size >::[reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) & it) |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[operator++](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator++)**() |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[operator++](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator++)**(int ) |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[operator--](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator--)**() |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) | **[operator--](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator--)**(int ) |
| type & | **[operator*](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator*)**() |
| type * | **[operator->](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator->)**() |
| type & | **[operator[]](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator[])**(int index) |
| bool | **[operator==](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator==)**(const [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) & rhs) |
| bool | **[operator!=](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#function-operator!=)**(const [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/) & rhs) |

## Friends

|                | Name           |
| -------------- | -------------- |
| class | **[CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1reverse__iterator/#friend-circulardelay)**  |

## Public Types Documentation

### typedef self_type

```cpp
typedef reverse_iterator CircularDelay< type, size >::reverse_iterator::self_type;
```


### typedef iterator_category

```cpp
typedef std::bidirectional_iterator_tag CircularDelay< type, size >::reverse_iterator::iterator_category;
```


### typedef difference_type

```cpp
typedef int CircularDelay< type, size >::reverse_iterator::difference_type;
```


## Public Functions Documentation

### function reverse_iterator

```cpp
inline reverse_iterator(
    const CircularDelay< type, size >::reverse_iterator & it
)
```


### function operator++

```cpp
inline self_type operator++()
```


### function operator++

```cpp
inline self_type operator++(
    int 
)
```


### function operator--

```cpp
inline self_type operator--()
```


### function operator--

```cpp
inline self_type operator--(
    int 
)
```


### function operator*

```cpp
inline type & operator*()
```


### function operator->

```cpp
inline type * operator->()
```


### function operator[]

```cpp
inline type & operator[](
    int index
)
```


### function operator==

```cpp
inline bool operator==(
    const self_type & rhs
)
```


### function operator!=

```cpp
inline bool operator!=(
    const self_type & rhs
)
```


## Friends

### friend CircularDelay

```cpp
friend class CircularDelay;
```


-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000