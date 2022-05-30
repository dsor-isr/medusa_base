---
title: console::CallbackEcho

---

# console::CallbackEcho



 [More...](#detailed-description)

Inherits from object

## Public Functions

|                | Name           |
| -------------- | -------------- |
| def | **[__init__](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#function---init--)**(self self, topic topic) |
| def | **[callback](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#function-callback)**(self self, data data, topic topic, current_time current_time =None) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| | **[topic](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-topic)**  |
| | **[count](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-count)**  |
| | **[prefix](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-prefix)**  |
| | **[suffix](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-suffix)**  |
| | **[done](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-done)**  |
| | **[str_fn](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-str-fn)**  |
| | **[first](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-first)**  |
| | **[last_msg_eval](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-last-msg-eval)**  |
| | **[last_topic](/medusa_base/api/markdown/medusa_addons/http_server/Classes/classconsole_1_1CallbackEcho/#variable-last-topic)**  |

## Detailed Description

```python
class console::CallbackEcho;
```




```
Callback instance that can print callback data in a variety of
formats. Used for all variants of rostopic echo
```

## Public Functions Documentation

### function __init__

```python
def __init__(
    self self,
    topic topic
)
```




```
:param plot: if ``True``, echo in plotting-friendly format, ``bool``
:param filter_fn: function that evaluates to ``True`` if message is to be echo'd, ``fn(topic, msg)``
:param echo_all_topics: (optional) if ``True``, echo all messages in bag, ``bool``
:param offset_time: (optional) if ``True``, display time as offset from current time, ``bool``
:param count: number of messages to echo, ``None`` for infinite, ``int``
:param field_filter_fn: filter the fields that are strified for Messages, ``fn(Message)->iter(str)``
```


### function callback

```python
def callback(
    self self,
    data data,
    topic topic,
    current_time current_time =None
)
```




```
Callback to pass to rospy.Subscriber or to call
manually. rospy.Subscriber constructor must also pass in the
topic name as an additional arg
:param data: Message
:param topic: topic name, ``str``
:param current_time: override calculation of current time, :class:`genpy.Time`
```


## Public Attributes Documentation

### variable topic

```python
topic;
```


### variable count

```python
count;
```


### variable prefix

```python
prefix;
```


### variable suffix

```python
suffix;
```


### variable done

```python
done;
```


### variable str_fn

```python
str_fn;
```


### variable first

```python
first;
```


### variable last_msg_eval

```python
last_msg_eval;
```


### variable last_topic

```python
last_topic;
```


-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000