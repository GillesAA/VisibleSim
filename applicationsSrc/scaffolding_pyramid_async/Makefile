# Selected desired b length
#B_LENGTH_DIR=b5/
B_LENGTH_DIR=b6/
#B_LENGTH_DIR=b7/
#B_LENGTH_DIR=b8/

.PHONY: test subdir $(B_LENGTH_DIR)

subdir: $(B_LENGTH_DIR)

$(B_LENGTH_DIR):
	@$(MAKE) -C $@ APPDIR="../$(APPDIR)" GLOBAL_INCLUDES="$(GLOBAL_INCLUDES)" GLOBAL_LIBS="$(GLOBAL_LIBS)" GLOBAL_CCFLAGS="$(GLOBAL_CCFLAGS)"

test:
	$(MAKE) -C $(B_LENGTH_DIR) test;

clean:
	$(MAKE) -C $(B_LENGTH_DIR) clean;
