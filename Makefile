CFLAGS = -Wall -O2

PREFIX=/usr/sbin
ALL=dcfshm

$(ALL): dcfshm.c
	$(CC) $(CFLAGS) -o $@ $<

install:
	mkdir -p $(DESTDIR)/$(PREFIX)
	cp dcfshm $(DESTDIR)/$(PREFIX)
	mkdir -p $(DESTDIR)/etc/init.d
	cp dcfshm.init $(DESTDIR)/etc/init.d/dcfshm

clean:
	rm -f $(ALL)
