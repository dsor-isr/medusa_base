---
title: CircularDelay::const_reverse_iterator

---

# CircularDelay::const_reverse_iterator





## Public Types

|                | Name           |
| -------------- | -------------- |
| typedef [const_reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) | **[self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#typedef-self-type)**  |
| typedef std::bidirectional_iterator_tag | **[iterator_category](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#typedef-iterator-category)**  |
| typedef int | **[difference_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#typedef-difference-type)**  |

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[const_reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-const-reverse-iterator)**(const [CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay/)< type, size >::[const_reverse_iterator](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) & it) |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) | **[operator++](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator++)**() |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) | **[operator++](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator++)**(int ) |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) | **[operator--](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator--)**() |
| [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) | **[operator--](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator--)**(int ) |
| const type & | **[operator*](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator*)**() |
| const type * | **[operator->](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator->)**() |
| const type & | **[operator[]](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator[])**(int index) |
| bool | **[operator==](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator==)**(const [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) & rhs) |
| bool | **[operator!=](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#function-operator!=)**(const [self_type](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/) & rhs) |

## Friends

|                | Name           |
| -------------- | -------------- |
| class | **[CircularDelay](/medusa_base/api/markdown/dsor_utils/dsor_utils/Classes/classCircularDelay_1_1const__reverse__iterator/#friend-circulardelay)**  |

## Public Types Documentation

### typedef self_type

```cpp
typedef const_reverse_iterator CircularDelay< type, size >::const_reverse_iterator::self_type;
```


### typedef iterator_category

```cpp
typedef std::bidirectional_iterator_tag CircularDelay< type, size >::const_reverse_iterator::iterator_category;
```


### typedef difference_type

```cpp
typedef int CircularDelay< type, size >::const_reverse_iterator::difference_type;
```


## Public Functions Documentation

### function const_reverse_iterator

```cpp
inline const_reverse_iterator(
    const CircularDelay< type, size >::const_reverse_iterator & it
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
inline const type & operator*()
```


### function operator->

```cpp
inline const type * operator->()
```


### function operator[]

```cpp
inline const type & operator[](
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