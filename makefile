TARGET = patriots

CC = gcc

ALLEGRO_LDFLAGS = $(shell pkg-config --libs allegro)
ALLEGRO_CFLAGS = $(shell pkg-config --cflags allegro)

CFLAGS = -Wall   $(ALLEGRO_CFLAGS)

LDFLAGS = -lpthread -lrt $(ALLEGRO_LDFLAGS) -lm

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
HEADERS = $(wildcard *.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
