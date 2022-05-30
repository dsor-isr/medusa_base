---
title: data_serializer_algorithms::DataSerializerAlgorithm

---

# data_serializer_algorithms::DataSerializerAlgorithm



## Functions

|                | Name           |
| -------------- | -------------- |
| def | **[payload_to_bytes](/medusa_base/api/markdown/medusa_comms/comms_acoustic/data_serializer/Namespaces/namespacedata__serializer__algorithms_1_1DataSerializerAlgorithm/#function-payload-to-bytes)**(bits_string bits_string) |
| def | **[payload_to_bits](/medusa_base/api/markdown/medusa_comms/comms_acoustic/data_serializer/Namespaces/namespacedata__serializer__algorithms_1_1DataSerializerAlgorithm/#function-payload-to-bits)**(bytes_string bytes_string) |
| def | **[dec_to_bin](/medusa_base/api/markdown/medusa_comms/comms_acoustic/data_serializer/Namespaces/namespacedata__serializer__algorithms_1_1DataSerializerAlgorithm/#function-dec-to-bin)**(dec_value dec_value, low low, high high, bits bits, is_int is_int =False) |
| def | **[bin_to_dec](/medusa_base/api/markdown/medusa_comms/comms_acoustic/data_serializer/Namespaces/namespacedata__serializer__algorithms_1_1DataSerializerAlgorithm/#function-bin-to-dec)**(bin_value bin_value, low low, high high, bits bits, is_int is_int =False) |
| def | **[extract_field](/medusa_base/api/markdown/medusa_comms/comms_acoustic/data_serializer/Namespaces/namespacedata__serializer__algorithms_1_1DataSerializerAlgorithm/#function-extract-field)**(msg_data msg_data, field_address field_address) |
| def | **[add_field_to_dict](/medusa_base/api/markdown/medusa_comms/comms_acoustic/data_serializer/Namespaces/namespacedata__serializer__algorithms_1_1DataSerializerAlgorithm/#function-add-field-to-dict)**(final_dict final_dict, field_address field_address, field_value field_value) |


## Functions Documentation

### function payload_to_bytes

```python
def payload_to_bytes(
    bits_string bits_string
)
```


### function payload_to_bits

```python
def payload_to_bits(
    bytes_string bytes_string
)
```


### function dec_to_bin

```python
def dec_to_bin(
    dec_value dec_value,
    low low,
    high high,
    bits bits,
    is_int is_int =False
)
```


### function bin_to_dec

```python
def bin_to_dec(
    bin_value bin_value,
    low low,
    high high,
    bits bits,
    is_int is_int =False
)
```


### function extract_field

```python
def extract_field(
    msg_data msg_data,
    field_address field_address
)
```


### function add_field_to_dict

```python
def add_field_to_dict(
    final_dict final_dict,
    field_address field_address,
    field_value field_value
)
```






-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000