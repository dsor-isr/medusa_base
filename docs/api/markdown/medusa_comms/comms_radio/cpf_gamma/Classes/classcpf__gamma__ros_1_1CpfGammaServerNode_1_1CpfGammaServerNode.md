---
title: cpf_gamma_ros::CpfGammaServerNode::CpfGammaServerNode

---

# cpf_gamma_ros::CpfGammaServerNode::CpfGammaServerNode





## Public Functions

|                | Name           |
| -------------- | -------------- |
| def | **[__init__](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function---init--)**(self self) |
| def | **[initializePublishers](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-initializepublishers)**(self self) |
| def | **[loadParams](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-loadparams)**(self self) |
| def | **[serverEnable](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-serverenable)**(self self) |
| def | **[parseData](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-parsedata)**(self self, data data) |
| def | **[cpfGammaMessage](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-cpfgammamessage)**(self self, parsed_data parsed_data) |
| def | **[etcpfGammaMessage](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-etcpfgammamessage)**(parsed_data parsed_data) |
| def | **[etcpfAckMessage](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#function-etcpfackmessage)**(parsed_data parsed_data) |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| | **[messages_type](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-messages-type)**  |
| | **[data_populate](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-data-populate)**  |
| | **[sock](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-sock)**  |
| | **[pubs](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-pubs)**  |
| | **[node_frequency](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-node-frequency)**  |
| | **[id](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-id)**  |
| | **[port](/medusa_base/api/markdown/medusa_comms/comms_radio/cpf_gamma/Classes/classcpf__gamma__ros_1_1CpfGammaServerNode_1_1CpfGammaServerNode/#variable-port)**  |

## Public Functions Documentation

### function __init__

```python
def __init__(
    self self
)
```




```
Constructor for ros node
```



```
###########################################################################################
@.@ Init node
###########################################################################################
```


### function initializePublishers

```python
def initializePublishers(
    self self
)
```


### function loadParams

```python
def loadParams(
    self self
)
```


### function serverEnable

```python
def serverEnable(
    self self
)
```


### function parseData

```python
def parseData(
    self self,
    data data
)
```


### function cpfGammaMessage

```python
def cpfGammaMessage(
    self self,
    parsed_data parsed_data
)
```


### function etcpfGammaMessage

```python
static def etcpfGammaMessage(
    parsed_data parsed_data
)
```


### function etcpfAckMessage

```python
static def etcpfAckMessage(
    parsed_data parsed_data
)
```


## Public Attributes Documentation

### variable messages_type

```python
messages_type;
```


### variable data_populate

```python
data_populate;
```


### variable sock

```python
sock;
```


### variable pubs

```python
pubs;
```


### variable node_frequency

```python
node_frequency;
```


### variable id

```python
id;
```


### variable port

```python
port;
```


-------------------------------

Updated on 2022-05-30 at 08:04:26 +0000