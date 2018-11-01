all: all_src all_base64 all_examples

all_src:
	$(MAKE) -C src
all_base64: all_src
	$(MAKE) -C base64
all_examples:
	$(MAKE) -C examples
	
clean: clean_src clean_base64 clean_include clean_examples
	rm -f *~ *.bak

clean_include:
	rm -f include/b64/*~

clean_src:
	$(MAKE) -C src clean;
clean_base64:
	$(MAKE) -C base64 clean;
clean_examples:
	$(MAKE) -C examples clean;
		
distclean: clean distclean_src distclean_base64 distclean_examples

distclean_src:
	$(MAKE) -C src distclean;
distclean_base64:
	$(MAKE) -C base64 distclean;
distclean_examples:
	$(MAKE) -C examples distclean;

