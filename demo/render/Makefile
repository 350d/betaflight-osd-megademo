# Host render of the OSD demo (see render.c)
SRC := ../../src/main
CFLAGS := -std=gnu11 -O1 -g -DSTM32F4 -DSTM32F411xE -DUSE_OSD_DEMO -Ifake -I. -I$(SRC) -w
OBJ := render.o hostenv.o chiptune.o pt3.o ay_synth.o chiptune_tracks.o maths.o printf.o typeconversion.o explog_approx.o

render: $(OBJ)
	$(CC) -o $@ $(OBJ) -lm

render.o: render.c $(SRC)/io/osd_demo.c $(SRC)/osd/osd_demo_font.inc hostenv.h fake/*.h
	$(CC) $(CFLAGS) -c render.c -o $@
hostenv.o: hostenv.c hostenv.h fake/*.h
	$(CC) $(CFLAGS) -c hostenv.c -o $@
chiptune.o: $(SRC)/io/chiptune.c
	$(CC) $(CFLAGS) -c $< -o $@
pt3.o: $(SRC)/io/pt3.c
	$(CC) $(CFLAGS) -c $< -o $@
ay_synth.o: $(SRC)/drivers/ay_synth.c
	$(CC) $(CFLAGS) -c $< -o $@
chiptune_tracks.o: $(SRC)/io/chiptune_tracks.c
	$(CC) $(CFLAGS) -c $< -o $@
maths.o: $(SRC)/common/maths.c
	$(CC) $(CFLAGS) -c $< -o $@
printf.o: $(SRC)/common/printf.c
	$(CC) $(CFLAGS) -c $< -o $@
typeconversion.o: $(SRC)/common/typeconversion.c
	$(CC) $(CFLAGS) -c $< -o $@
clean:
	rm -f *.o render
explog_approx.o: $(SRC)/common/explog_approx.c
	$(CC) $(CFLAGS) -c $< -o $@
