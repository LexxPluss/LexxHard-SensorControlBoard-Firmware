SHELL := /bin/bash
.SHELLFLAGS := -o pipefail -c

.PHONY: run run_new_standard run_new_push run_all clean distclean setup

OLD_COMMIT := 8c94ed67dfa3e68e0c2aa007839b23dca4be1262
BUILD_DIR := build
OLD_SRC_DIR := $(BUILD_DIR)/old_src

CXX := g++
CXXFLAGS := -std=c++20 -Wall -Wextra -I. -I./stubs -I$(OLD_SRC_DIR)
LDFLAGS :=

TEST_BINARY := $(BUILD_DIR)/test_runner
TEST_SOURCES := test_runner.cpp stubs/zephyr_stubs.cpp stubs/lexxhard_stubs.cpp \
                $(OLD_SRC_DIR)/bmu_lipy041_decode.cpp

run: $(TEST_BINARY)
	@echo "Running board controller baseline tests..."
	@$(TEST_BINARY)
	@echo "Tests complete."

$(TEST_BINARY): setup $(TEST_SOURCES) stubs/zephyr_stubs.hpp stubs/lexxhard_stubs.hpp \
                 $(OLD_SRC_DIR)/board_controller.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(TEST_SOURCES) -o $(TEST_BINARY) $(LDFLAGS)

setup: $(OLD_SRC_DIR)/board_controller.cpp $(OLD_SRC_DIR)/board_controller.hpp \
       $(OLD_SRC_DIR)/common.hpp $(OLD_SRC_DIR)/power_state.hpp \
       $(OLD_SRC_DIR)/adc_reader.hpp $(OLD_SRC_DIR)/can_controller.hpp \
       $(OLD_SRC_DIR)/led_controller.hpp \
       $(OLD_SRC_DIR)/bmu_lipy041_decode.hpp $(OLD_SRC_DIR)/bmu_lipy041_decode.cpp

$(OLD_SRC_DIR)/%.cpp:
	@mkdir -p $(OLD_SRC_DIR)
	git show $(OLD_COMMIT):lexxpluss_apps/src/$*.cpp > $@

$(OLD_SRC_DIR)/%.hpp:
	@mkdir -p $(OLD_SRC_DIR)
	git show $(OLD_COMMIT):lexxpluss_apps/src/$*.hpp > $@

# New (working tree) sources, read directly from ../../src
NEW_SRC_DIR := ../../src
NEW_CXXFLAGS := -std=c++20 -Wall -Wextra -I. -I./stubs -I$(NEW_SRC_DIR) \
                -DBC_NEW_BUILD -DBC_SOURCE='"$(NEW_SRC_DIR)/board_controller.cpp"'
NEW_SOURCES := test_runner.cpp stubs/zephyr_stubs.cpp stubs/lexxhard_stubs.cpp \
               $(NEW_SRC_DIR)/bmu_lipy041_decode.cpp $(NEW_SRC_DIR)/auto_charge_transitions.cpp
NEW_DEPS := $(NEW_SOURCES) stubs/zephyr_stubs.hpp stubs/lexxhard_stubs.hpp $(wildcard $(NEW_SRC_DIR)/*.hpp) \
            $(NEW_SRC_DIR)/board_controller.cpp

$(BUILD_DIR)/test_new_standard: $(NEW_DEPS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(NEW_CXXFLAGS) -DBC_NEW_BUILD_NAME='"standard"' $(NEW_SOURCES) -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_new_push: $(NEW_DEPS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(NEW_CXXFLAGS) -DENABLE_PUSH_MODE=1 -DBC_NEW_BUILD_NAME='"push"' $(NEW_SOURCES) -o $@ $(LDFLAGS)

run_new_standard: $(BUILD_DIR)/test_new_standard
	@$(BUILD_DIR)/test_new_standard

run_new_push: $(BUILD_DIR)/test_new_push
	@$(BUILD_DIR)/test_new_push

run_all: $(TEST_BINARY) $(BUILD_DIR)/test_new_standard $(BUILD_DIR)/test_new_push
	@$(TEST_BINARY) | sed -n 's/^RESULT: /old: RESULT: /p'
	@$(BUILD_DIR)/test_new_standard | sed -n 's/^RESULT: /new_standard: /p'
	@$(BUILD_DIR)/test_new_push | sed -n 's/^RESULT: /new_push: /p'

clean:
	rm -f $(TEST_BINARY) $(BUILD_DIR)/test_new_standard $(BUILD_DIR)/test_new_push $(BUILD_DIR)/*.o

distclean:
	rm -rf $(BUILD_DIR)
