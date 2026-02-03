SHELL := /bin/sh

COLCON_DIRS := build install log

.PHONY: help build clean clean-colcon clean-pycache clean-logs

help:
	@printf "%s\n" \
	  "make clean        Remove colcon build artifacts, python caches, temp/log files" 

build:
	@echo "Build target - not implemented"
	colcon build 

clean:
	@$(MAKE) clean-colcon
	@$(MAKE) clean-pycache
	@$(MAKE) clean-logs

clean-colcon:
	rm -rf $(COLCON_DIRS)

clean-pycache:
	find . -type d -name '__pycache__' -prune -exec rm -rf {} +
	find . -type f -name '*.py[co]' -delete
	find . -type f -name '*$py.class' -delete
	find . -type f -name '.Python' -delete

clean-logs:
	find . -type f -name '*.log' -delete
	find . -type f -name '*.tmp' -delete
	find . -type f -name '*.bak' -delete

