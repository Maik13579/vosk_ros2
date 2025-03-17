#!/bin/bash
# Usage: ./build_fst.sh input.jsgf /opt/vosk_model/big_speech_model
# This script converts a JSGF file into an FSM and compiles it into an FST.
# Idea for this script from https://github.com/alphacep/vosk-api/issues/55

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 input.jsgf /path/to/base_model"
    exit 1
fi

JSGF_FILE="$1"
BASE_MODEL_DIR="$2"

# Determine new model name from the JSGF file (e.g., "demo" from demo.jsgf)
NEW_MODEL_NAME=$(basename "$JSGF_FILE" .jsgf)
echo "New model name: $NEW_MODEL_NAME"

# Determine the parent directory of the base model and create a new model directory there.
PARENT_DIR=$(dirname "$BASE_MODEL_DIR")
NEW_MODEL_DIR="${PARENT_DIR}/${NEW_MODEL_NAME}"

echo "Copying base model from $BASE_MODEL_DIR to $NEW_MODEL_DIR..."
cp -r "$BASE_MODEL_DIR" "$NEW_MODEL_DIR"
if [ $? -ne 0 ]; then
    echo "Error copying model directory."
    exit 1
fi

# Use the words symbol table from the new model's graph folder.
WORDS_FILE="${NEW_MODEL_DIR}/graph/words.txt"
if [ ! -f "$WORDS_FILE" ]; then
    echo "Error: Symbol table not found at $WORDS_FILE"
    exit 1
fi

echo "Using symbol table: $WORDS_FILE"

# Sort the words file numerically by the second column (ID) and remove duplicates.
sort -k2,2n -u "$WORDS_FILE" -o "$WORDS_FILE"

# Convert JSGF to FSM.
BASE_NAME=$(basename "$JSGF_FILE" .jsgf)
FSM_FILE="${BASE_NAME}.fsm"
echo "Converting JSGF ($JSGF_FILE) to FSM ($FSM_FILE)..."
sphinx_jsgf2fsg -jsgf "$JSGF_FILE" -fsm "$FSM_FILE"
if [ $? -ne 0 ]; then
    echo "Error converting JSGF to FSM"
    exit 1
fi

echo "Replacing 'unk_arc' with '[unk]' in FSM file..."
sed -i 's/unk_arc/\[unk\]/g' "$FSM_FILE"

# Detect missing words
echo "Checking for missing words..."
awk '{print $3}' "$FSM_FILE" | sort | uniq > fsm_words.txt
awk '{print $1}' "$WORDS_FILE" | sort | uniq > words_dict.txt
MISSING_WORDS=$(comm -23 fsm_words.txt words_dict.txt)

if [ -n "$MISSING_WORDS" ]; then
    echo "Missing words: $MISSING_WORDS"
    echo "With this script you can only use grammars with known words."
    echo "Check the documentation for more information. https://alphacephei.com/vosk/adaptation"
    exit 1
fi
# Compile FSM to FST using the updated symbol table.
FST_FILE="Gr.fst"
echo "Compiling FSM to FST..."

fstcompile --acceptor --isymbols="$WORDS_FILE" --osymbols="$WORDS_FILE" \
  --keep_isymbols=true --keep_osymbols=true "$FSM_FILE"  \
  | fstdeterminize | fstminimize | fstrmepsilon | fstarcsort  > "$FST_FILE"


echo "Copying compiled FST to ${NEW_MODEL_DIR}/graph/Gr.fst..."
cp "$FST_FILE" "${NEW_MODEL_DIR}/graph/Gr.fst"
if [ $? -ne 0 ]; then
    echo "Error copying Gr.fst to new model directory."
    exit 1
fi

echo "FST generated and installed successfully in ${NEW_MODEL_DIR}/graph/Gr.fst"
