---
title: CircularDelay::iterator

---

# CircularDelay::iterator





## Public Types

|                | Name           |
| -------------- | -------------- |
| typedef [iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#typedef-self-type)**  |
| typedef std::bidirectional_iterator_tag | **[iterator_category](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#typedef-iterator-category)**  |
| typedef int | **[difference_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#typedef-difference-type)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-iterator)**(const [CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/)< type, size >::[iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) & it) |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[operator++](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator++)**() |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[operator++](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator++)**(int ) |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[operator--](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator--)**() |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) | **[operator--](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator--)**(int ) |
| type & | **[operator*](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator*)**() |
| type * | **[operator->](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator->)**() |
| type & | **[operator[]](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator[])**(unsigned int index) |
| bool | **[operator==](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator==)**(const [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) & rhs) |
| bool | **[operator!=](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#function-operator!=)**(const [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/) & rhs) |

## Friends

|                | Name           |
| -------------- | -------------- |
| class | **[CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1iterator/#friend-circulardelay)**  |

## Public Types Documentation

### typedef self_type

```cpp
typedef iterator CircularDelay< type, size >::iterator::self_type;
```


### typedef iterator_category

```cpp
typedef std::bidirectional_iterator_tag CircularDelay< type, size >::iterator::iterator_category;
```


### typedef difference_type

```cpp
typedef int CircularDelay< type, size >::iterator::difference_type;
```


## Public Functions Documentation

### function iterator

```cpp
inline iterator(
    const CircularDelay< type, size >::iterator & it
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
    unsigned int index
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