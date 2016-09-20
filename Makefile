CFLAGS = -Wall -O2

dcfshm: dcfshm.c
	$(CC) $(CFLAGS) -o $@ $<
