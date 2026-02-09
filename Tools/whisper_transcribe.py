#!/usr/bin/env python3
"""
Simple local transcription helper for the project.

Usage:
  python whisper_transcribe.py <out_file> [duration_seconds] [model]
  python whisper_transcribe.py <out_file> --stream --stop-file <flag_path> [--max-seconds N] [model]

- Records from the default microphone for `duration_seconds` (default 5s)
  and writes transcription (plain text) to `out_file`.
- By default it attempts to use `faster-whisper` with model name "small"
  on CPU. Install with:

  pip install sounddevice soundfile faster-whisper

- For better performance or if you prefer another backend, modify this script.

Note: This script is a starting point. Model download or first-run may be
slow. For production use, consider pre-downloading a GGML/Whisper model
and using whisper.cpp or a system-optimized pipeline.
"""

import argparse
import sys
import os
import tempfile
import queue
import time

try:
    import sounddevice as sd
    import soundfile as sf
except Exception as e:
    sys.exit(f"Missing audio dependencies: {e}")

try:
    from faster_whisper import WhisperModel
except Exception as e:
    # We'll still allow the script to fail with helpful message
    WhisperModel = None


def record_to_file(filename, duration, samplerate=16000, channels=1):
    data = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=channels, dtype='int16')
    sd.wait()
    sf.write(filename, data, samplerate, subtype='PCM_16')


def record_stream_to_file(filename, stop_file, samplerate=16000, channels=1, max_seconds=300):
    """
    Record microphone audio until a stop flag appears or max_seconds elapse.
    This keeps memory usage low by streaming to disk.
    """
    if not stop_file:
        raise ValueError("stop_file is required for streaming mode")

    if os.path.exists(stop_file):
        os.remove(stop_file)

    q = queue.Queue()

    def callback(indata, frames, time_info, status):
        if status:
            print(status, file=sys.stderr)
        q.put(indata.copy())

    start = time.time()
    with sf.SoundFile(filename, mode='w', samplerate=samplerate, channels=channels, subtype='PCM_16') as file:
        with sd.InputStream(samplerate=samplerate, channels=channels, dtype='int16', callback=callback):
            while True:
                try:
                    data = q.get(timeout=0.2)
                    file.write(data)
                except queue.Empty:
                    pass

                if stop_file and os.path.exists(stop_file):
                    break
                if max_seconds and (time.time() - start) >= max_seconds:
                    print("Max duration reached, stopping.", file=sys.stderr)
                    break


def transcribe_with_faster_whisper(model_name, audio_path):
    if WhisperModel is None:
        raise RuntimeError("faster-whisper is not installed. Run: pip install faster-whisper")

    # Use CPU and int8 quantization where available
    model = WhisperModel(model_name, device="cpu", compute_type="int8")
    segments, info = model.transcribe(audio_path, beam_size=5)
    text = []
    for segment in segments:
        text.append(segment.text)
    return "\n".join(text)


def main():
    parser = argparse.ArgumentParser(description="Record audio and transcribe with local Whisper (faster-whisper)")
    parser.add_argument('out_file', help='Path to write transcription output (text)')
    parser.add_argument('duration', nargs='?', type=float, default=5.0, help='Record duration in seconds (default 5)')
    parser.add_argument('model', nargs='?', default='small', help='Model name or path for faster-whisper (default: small)')
    parser.add_argument('--stream', action='store_true', help='Record until stop-file appears (or max-seconds elapse)')
    parser.add_argument('--stop-file', default=None, help='Flag file to stop streaming mode')
    parser.add_argument('--max-seconds', type=float, default=300.0, help='Safety cutoff for streaming mode')
    parser.add_argument('--save-audio', default=None, help='Optional path to keep recorded audio (defaults to temp file)')
    args = parser.parse_args()

    out_file = args.out_file
    duration = args.duration
    model = args.model

    try:
        if args.stream:
            audio_path = args.save_audio or os.path.splitext(out_file)[0] + "_audio.wav"
            print(f"Recording (stream) to {audio_path}... stop file: {args.stop_file}")
            record_stream_to_file(audio_path, args.stop_file, max_seconds=args.max_seconds)
            print("Recording complete. Transcribing...")
            text = transcribe_with_faster_whisper(model, audio_path)
            with open(out_file, 'w', encoding='utf-8') as f:
                f.write(text)
            print("Transcription written to", out_file)
            if args.stop_file and os.path.exists(args.stop_file):
                os.remove(args.stop_file)
            if not args.save_audio and os.path.exists(audio_path):
                os.remove(audio_path)
        else:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
                tmpname = tmp.name
            print(f"Recording {duration}s to {tmpname}...")
            record_to_file(tmpname, duration)
            print("Recording complete. Transcribing...")
            text = transcribe_with_faster_whisper(model, tmpname)
            with open(out_file, 'w', encoding='utf-8') as f:
                f.write(text)
            print("Transcription written to", out_file)
            os.remove(tmpname)
        return 0
    except Exception as e:
        try:
            with open(out_file, 'w', encoding='utf-8') as f:
                f.write(f"ERROR: {e}\n")
        except Exception:
            pass
        print("ERROR:", e, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
