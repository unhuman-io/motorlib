#$(shell > tuneable.txt) 

BUILD_DIR = build

all: $(BUILD_DIR)/tuneable-plugin.so ##$(BUILD_DIR)/tuneable.cpp

$(BUILD_DIR):
	mkdir -p $@


# $(BUILD_DIR)/tuneable.cpp: $(OBJECTS) 
# 	sed -nE 's/(\S*)\s(\S+)\s(\S+)/api.add_variable("\3", new \1 API\2(\&config::\3));/p' tuneable.txt > tuneable.cpp 

$(BUILD_DIR)/tuneable-plugin.so: tuneable_attribute_plugin.cpp | $(BUILD_DIR) 
	g++ -I`g++ -print-file-name=plugin`/include -fPIC -x c++ -shared $< -o $(BUILD_DIR)/tuneable-plugin.so -g -pedantic 
