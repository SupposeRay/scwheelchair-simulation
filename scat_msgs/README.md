# scat_msgs

## Brief description

Package containing custom messages and services. 

- EnvObjectList.msg contains a header and list of EnvObjects, and is used to communicate all recognized objects in the environment. 
- EnvObject.msg is a very general description of any object, based on an ID and list of parameters.
The meaning of the list of parameters depends on ID. These are documented in the object_definition.xlsx file in the docs folder
