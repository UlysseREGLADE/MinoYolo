CC=g++
IDIR =./include
SDIR =./src
CFLAGS= -I 	$(IDIR)
ODIR=obj
LIBS=


_OBJ = main.o strategy.o L6470Driver.o  SPI-Wrapper.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)


.PHONY: clean


clean:
	rm -f $(ODIR)/*.o *~ main
