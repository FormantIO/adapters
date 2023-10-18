# ZMQ Adapter README

## Usage
1. Run `./build.sh`
    - `chmod u+x build.sh`, if necessary permissions haven't been granted
    - A zmq_adapter.zip containing the adapter will be generated in the root folder
2. Create a new adapter in https://app.formant.io/adapter/new
    - Give it an appropriate name (say zmq_test)
    - The exec command to be given as `./start.sh`
    - Upload the zip file generated in the previous step
3. Access settings of your formant device and scroll down to **Adapter Configuration**
4. Click on **Add Adapter** and select the given name (zmq_test)
5. Edit configuration
    - port_start and port_end are mandatory
6. Save Changes
7. Agent will reload in the local device
8. Run production tests, if need be


## Tests (Production)
1. Ensure the adapter zip is uploaded and agent is installed in the local device
2. Run the test from the root folder (zmq_adapter)
    - For example, `python tests/test_publish.py`
    - For commands, run `python tests/test_command.py` and issue the command (set on test) from Formant cloud

## Tests (Local)
1. Comment out the ```self.configuration_validator = ...``` line
2. Add the following lines
    ```Adapter.config = Config(json.loads(open("./tests/config.json").read())[CONFIG_NAME])
        Adapter.is_config_initialized = True```
    before self.configuration_validator
    also add `import json` at the top