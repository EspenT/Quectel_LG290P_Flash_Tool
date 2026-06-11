# Build quectool (Homebrew libserialport on macOS)
PREFIX ?= /opt/homebrew

quectool: quectool.c
	gcc quectool.c -o quectool -I$(PREFIX)/include -L$(PREFIX)/lib -lserialport

clean:
	rm -f quectool

.PHONY: clean
