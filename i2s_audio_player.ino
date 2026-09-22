/*
  TUI 2026 — record on one object, play on another
  SKETCH B — THE PLAYER (the object with the speaker)

  This board makes its own small WiFi network and waits. When the recorder
  sends a clip, it lands in PSRAM and stays there. Nothing is heard until the
  recorder reports that the cube has been opened while it was dialled to
  LISTEN — then the clip plays through once and stops. Any other reading
  that arrives while it is playing (including the cube closing again) stops
  it early, the same as if it had simply finished.

  There is no button on this board any more: the recorder owns the cube's
  rotary sensor and pushes readings over WiFi, and this board decides what
  they mean. See "Receiving" below for the two message types.

  Keeping the arrival silent is deliberate: the object holds what it was given
  until a person asks for it, rather than announcing itself in an empty room.

  It is the player that runs the network, not the recorder, and that is on
  purpose: it means the recorder can simply push a finished clip — or a cube
  reading — at a fixed address the moment it has one. Nobody has to poll, and
  nothing has to know anybody's IP address.

  Do A_bench_test first. It proves the amplifier and speaker work without any
  of this.

  What the LED tells you:
      slow blink      no clip yet — nothing to play
      off             a clip is loaded and waiting for the cube to open
      steady on       playing
      fast flicker    a clip is arriving

  Board:  Arduino Nano ESP32 (needs PSRAM).
  Parts:  an I2S output board, a speaker. Playback is triggered entirely over
          WiFi by the recorder's cube-position messages — see CUBE_OPEN_LO/HI
          and CUBE_LISTEN_LO/HI below. Set OUTPUT_BOARD below to whichever
          output board you have.

  Wiring — MAX98357A (mono amplifier, drives a speaker directly):
      Nano ESP32       MAX98357A
      VBUS ──────────  VIN     (5 V, but ONLY on USB power — see README)
      GND  ──────────  GND
      D2   ──────────  BCLK
      D3   ──────────  LRC
      D5   ──────────  DIN
                       SPK+ / SPK-  ── speaker

  Wiring — PCM5100 / PCM5102 (stereo line-level DAC, NO amplifier on board):
      Nano ESP32       PCM510x module
      3V3  ──────────  VIN
      GND  ──────────  GND
      D2   ──────────  BCK
      D3   ──────────  LCK  (also labelled LRCK or WS)
      D5   ──────────  DIN
                       SCK  ── GND    must be GND, not floating. This is what
                                      switches on the internal clock generator
                                      so no fourth clock wire is needed.
                       XSMT ── 3V3    un-mutes it. Low is silence.
                       FMT  ── GND    selects I2S rather than left-justified
                       FLT, DEMP ── GND
                       L / R / GND ──> powered speaker, or an amplifier
      This chip CANNOT drive a speaker. See the README.

  Wiring — white "now playing" LED (optional, on its own pin so it is
  independent of the LED_BUILTIN status codes above):
      Nano ESP32       LED
      D6   ────────── 220Ω resistor ── LED anode (long leg, +)
      GND  ────────── LED cathode (short leg, –)
*/

#include <Arduino.h>

// The ESP_I2S library arrived with ESP32 board package 3.0. On a 2.x package
// the include below fails with "ESP_I2S.h: No such file or directory". See the
// README section "If ESP_I2S.h is not found".
#ifndef ESP_ARDUINO_VERSION_MAJOR
#error "ESP32 board package not detected. Tools > Board > select Arduino Nano ESP32."
#elif ESP_ARDUINO_VERSION_MAJOR < 3
#error "Needs ESP32 board package 3.0 or newer. Boards Manager > install 'esp32' by Espressif Systems, then pick Arduino Nano ESP32 under it."
#endif

#include <ESP_I2S.h>
#include <WiFi.h>

// ---------- things you might want to change ----------

// Must match B_recorder exactly. At least 8 characters for the password.
const char *LINK_SSID = "tui-audio";
const char *LINK_PASS = "tui2026bergamo";

const int I2S_BCLK = D2;      // to amp BCLK
const int I2S_WS   = D3;      // to amp LRC
const int I2S_DOUT = D5;      // to amp DIN

const int LED_PLAY_PIN = D6;   // white LED: on while a clip is playing, off otherwise

// The recorder's rotary cube sensor reports a raw reading in one of three
// bands. This board only ever acts on two of them — RECORD is listed purely
// so the bands read as one shared protocol with the recorder's own code.
const uint16_t CUBE_RECORD_LO = 800,  CUBE_RECORD_HI = 1200;   // not used here
const uint16_t CUBE_OPEN_LO   = 2700, CUBE_OPEN_HI   = 3100;
const uint16_t CUBE_LISTEN_LO = 3900, CUBE_LISTEN_HI = 4300;

const uint32_t SAMPLE_RATE = 16000;   // must match the recorder
const uint32_t MAX_SECONDS = 60;      // must be at least what the recorder sends

// What opening the cube does while playback is already running.
//   false  play the clip through once, then stop. Open it again to hear it again.
//   true   keep looping until the cube closes.
const bool LOOP_FOREVER = false;

// How loud the output is. 1.0 is full scale.
//
// SET THIS TO ABOUT 0.1 IF YOU ARE LISTENING ON HEADPHONES. The PCM510x puts
// out 2.1 Vrms, which is line level — roughly ten times what headphones expect,
// and very loud indeed on your head. See the README.
const float OUTPUT_GAIN = 0.3;


// Which board is on the end of the I2S bus.
//   0 = MAX98357A   mono amplifier, speaker straight off it
//   1 = PCM5100 / PCM5102   stereo line-level DAC, needs a powered speaker
//
// The stored clip stays mono either way. For the DAC the same sample is sent
// to both channels on its way out, so it comes out of both sides rather than
// only the left.
#define OUTPUT_BOARD 1

// -----------------------------------------------------

const uint16_t LINK_PORT = 5001;

const size_t WAV_HEADER_BYTES = 44;
const size_t BYTES_PER_SECOND = SAMPLE_RATE * 2;
const size_t CLIP_CAPACITY    = WAV_HEADER_BYTES + BYTES_PER_SECOND * MAX_SECONDS;

// 64 ms of audio per write. This number IS the stop latency: a cube-closed
// message is only acted on once per chunk, so a smaller chunk means a
// snappier stop and more trips round the loop. Below about 512 it starts to
// stutter.
const size_t PLAY_CHUNK = 2048;

const unsigned long RECEIVE_TIMEOUT_MS = 10000;   // silence before we give up

#if OUTPUT_BOARD == 1
// One mono 16-bit sample becomes two 32-bit ones, so four times the bytes.
static uint8_t rawOut[PLAY_CHUNK * 4];
#endif

I2SClass  i2s;
WiFiServer server(LINK_PORT);

// Two buffers, so a clip that arrives half way through does not destroy the
// one we already have. We only swap them once the whole thing is safely here.
uint8_t *clip     = nullptr;   // the one we play
uint8_t *incoming = nullptr;   // the one being received
size_t   clipBytes = 0;

bool   playing = false;
size_t playPos = WAV_HEADER_BYTES;

// The cube, as last reported by the recorder.
bool cubeOpen    = false;   // is the latest reading inside the OPEN band right now?
bool listenArmed = false;   // was the last non-open reading inside the LISTEN band?

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // This board makes its own USB port, so the port vanishes and comes back
  // every time the board resets, and the Serial Monitor needs about a second
  // to reattach. Anything printed before that is simply lost — which is why
  // setup() can look silent while loop() prints fine.
  //
  // So wait for the monitor, but with a deadline: a bare "while (!Serial);"
  // has none, and on a board running from a battery with nothing plugged in
  // it would sit there forever looking dead.
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) { }
  delay(100);   // the port is open; give the monitor a moment to start listening

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PLAY_PIN, OUTPUT);

  if (ESP.getPsramSize() == 0) {
    haltWith("No PSRAM. Check the board really is an Arduino Nano ESP32.");
  }
  clip     = (uint8_t *) ps_malloc(CLIP_CAPACITY);
  incoming = (uint8_t *) ps_malloc(CLIP_CAPACITY);
  if (clip == nullptr || incoming == nullptr) {
    haltWith("Could not reserve the two clip buffers in PSRAM.");
  }

  i2s.setPins(I2S_BCLK, I2S_WS, I2S_DOUT, -1);     // -1: no microphone here
#if OUTPUT_BOARD == 1
  // 32-bit stereo slots put 64 bit-clocks in every frame. The PCM510x accepts
  // 32, 48 or 64 of them, but its internal clock generator is only documented
  // as working at 64 for a 16 kHz sample rate, so that is what we use.
  bool ok = i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                      I2S_SLOT_MODE_STEREO);
#else
  bool ok = i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT,
                      I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT);
#endif
  if (!ok) haltWith("I2S would not start. Check the pin numbers.");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(LINK_SSID, LINK_PASS);
  WiFi.setSleep(false);
  server.begin();

  Serial.printf("Network '%s' is up at %s, listening on port %u\n",
                LINK_SSID, WiFi.softAPIP().toString().c_str(), LINK_PORT);
}

void loop() {
  acceptIncomingMessage();
  servePlayback();
  showStatusOnLed();
  updatePlayLed();
}

// ---------------------------------------------------------------------------
// Receiving
//
// Every connection starts with a 4-byte magic. Two kinds arrive on the same
// port:
//
//   "TUI1" | 4-byte length, smallest byte first | that many bytes of clip
//   — a finished recording. We answer 'K' if we got it all, 'X' if not.
//
//   "TUI2" | 2-byte cube reading, smallest byte first
//   — the recorder's rotary sensor, sent whenever it changes. We answer 'K'
//   once we've read it. See CUBE_OPEN_LO/HI and CUBE_LISTEN_LO/HI above for
//   what the reading means.

void acceptIncomingMessage() {
  WiFiClient client = server.available();
  if (!client) return;

  uint8_t magic[4];
  if (!readExactly(client, magic, 4)) {
    Serial.println("connection dropped before we saw a full header, ignoring");
    client.stop();
    return;
  }

  if (memcmp(magic, "TUI1", 4) == 0) {
    receiveClip(client);
  } else if (memcmp(magic, "TUI2", 4) == 0) {
    receiveCubeState(client);
  } else {
    Serial.println("not one of ours, ignoring");
    client.stop();
  }
}

void receiveClip(WiFiClient &client) {
  Serial.println("someone is connecting with a clip");
  playing = false;                     // go quiet while the new one arrives

  uint8_t lenBytes[4];
  if (!readExactly(client, lenBytes, 4)) {
    Serial.println("clip header cut off, ignoring");
    client.stop();
    return;
  }

  size_t want = (size_t)lenBytes[0]
              | ((size_t)lenBytes[1] <<  8)
              | ((size_t)lenBytes[2] << 16)
              | ((size_t)lenBytes[3] << 24);

  if (want <= WAV_HEADER_BYTES || want > CLIP_CAPACITY) {
    Serial.printf("clip is %u bytes, which does not fit — refusing\n", (unsigned)want);
    client.write('X');
    client.stop();
    return;
  }

  Serial.printf("receiving %u bytes", (unsigned)want);

  size_t        got     = 0;
  unsigned long lastRxMs = millis();
  while (got < want) {
    int n = client.read(incoming + got, want - got);
    if (n > 0) {
      got += n;
      lastRxMs = millis();
    } else {
      if (!client.connected() && !client.available()) break;
      if (millis() - lastRxMs > RECEIVE_TIMEOUT_MS) break;
      delay(1);                        // a yield, not a pause
    }
    digitalWrite(LED_BUILTIN, (millis() / 60) % 2);
  }

  if (got == want) {
    // Swap the buffers. The old clip becomes the space the next one lands in.
    uint8_t *previous = clip;
    clip      = incoming;
    incoming  = previous;
    clipBytes = want;

    client.write('K');
    client.flush();
    Serial.printf(" — ok, %.1f seconds. Stored. Open the cube on LISTEN to play it.\n",
                  (want - WAV_HEADER_BYTES) / (float)BYTES_PER_SECOND);
  } else {
    client.write('X');
    client.flush();
    Serial.printf(" — only %u arrived, keeping the old clip\n", (unsigned)got);
  }
  client.stop();
}

// A cube-position update from the recorder. We only track two things from
// it: whether the cube is open right now, and whether the last time it was
// NOT open, it was dialled to LISTEN. Opening the cube while armed to listen
// starts playback; any reading that isn't "still open" while playing stops
// it — including the cube closing again.
void receiveCubeState(WiFiClient &client) {
  uint8_t valueBytes[2];
  if (!readExactly(client, valueBytes, 2)) {
    Serial.println("cube-state message cut off, ignoring");
    client.stop();
    return;
  }
  uint16_t value = (uint16_t)valueBytes[0] | ((uint16_t)valueBytes[1] << 8);

  bool wasOpen = cubeOpen;
  cubeOpen = (value >= CUBE_OPEN_LO && value <= CUBE_OPEN_HI);

  if (!cubeOpen) {
    // Not in the open band right now, so this reading tells us the mode the
    // cube is dialled to. Only LISTEN matters here — anything else
    // (RECORD, or values in between) disarms playback until LISTEN shows up
    // again.
    listenArmed = (value >= CUBE_LISTEN_LO && value <= CUBE_LISTEN_HI);
  }

  if (cubeOpen && !wasOpen) {
    if (listenArmed && !playing) {
      Serial.println("cube opened on LISTEN");
      startPlayback();
    } else if (!listenArmed) {
      Serial.println("cube opened, but not armed to listen — ignoring");
    }
  } else if (!cubeOpen && wasOpen && playing) {
    playing = false;
    Serial.println("cube closed — stopped");
  }

  client.write('K');
  client.flush();
  client.stop();
}

bool readExactly(WiFiClient &client, uint8_t *into, size_t count) {
  size_t        got = 0;
  unsigned long startMs = millis();
  while (got < count) {
    int n = client.read(into + got, count - got);
    if (n > 0) got += n;
    else if (millis() - startMs > 3000) return false;
    else delay(1);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Playback

void startPlayback() {
  if (clipBytes <= WAV_HEADER_BYTES) {
    // Worth saying out loud: a cube that opens and nothing happens looks
    // broken, and "no clip yet" is the commonest reason for it on the first
    // afternoon.
    Serial.println("nothing stored yet — record something and send it first");
    return;
  }
  playPos = WAV_HEADER_BYTES;
  playing = true;
  Serial.printf("playing %.1f seconds%s\n",
                (clipBytes - WAV_HEADER_BYTES) / (float)BYTES_PER_SECOND,
                LOOP_FOREVER ? ", looping until the cube closes" : "");
}

void servePlayback() {
  if (!playing) return;

  size_t left = clipBytes - playPos;
  size_t n    = left < PLAY_CHUNK ? left : PLAY_CHUNK;   // bytes of mono 16-bit

#if OUTPUT_BOARD == 1
  // Widen to 32 bits and copy each sample into both channels.
  const int16_t *in  = (const int16_t *)(clip + playPos);
  int32_t       *out = (int32_t *)rawOut;
  size_t samples = n / 2;
  for (size_t i = 0; i < samples; i++) {
    float v = (float)in[i] * OUTPUT_GAIN * 65536.0;
    if (v >  2147418112.0) v =  2147418112.0;
    if (v < -2147483648.0) v = -2147483648.0;
    int32_t wide = (int32_t)v;
    out[i * 2]     = wide;      // left
    out[i * 2 + 1] = wide;      // right
  }
  i2s.write(rawOut, samples * 8);
#else
  i2s.write(clip + playPos, n);
#endif
  playPos += n;

  if (playPos >= clipBytes) {
    if (LOOP_FOREVER) {
      playPos = WAV_HEADER_BYTES;      // straight back to the beginning
    } else {
      playing = false;
      Serial.println("finished");
    }
  }
}

// ---------------------------------------------------------------------------
// Helpers

void showStatusOnLed() {
  if (clipBytes <= WAV_HEADER_BYTES) {
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);   // waiting for anything
  } else {
    digitalWrite(LED_BUILTIN, playing ? HIGH : LOW);
  }
}

// A dedicated white LED that simply tracks playback: on while a clip is
// playing, off the rest of the time. Separate from LED_BUILTIN above, which
// carries the fuller status code (waiting for a clip / arriving / playing).
void updatePlayLed() {
  digitalWrite(LED_PLAY_PIN, playing ? HIGH : LOW);
}

void haltWith(const char *message) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    Serial.println(message);
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(850);
  }
}
