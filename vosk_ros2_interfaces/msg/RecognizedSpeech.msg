Result[] results            # List of recognized words with timestamps/confidence
string text                 # Complete recognized text from this chunk
int32 spk                   # Speaker ID (if speaker recognition is enabled)
Alternative[] alternatives  # Alternate transcriptions for this speech segment
float32 elapsed_time        # Time elapsed (in seconds) for this recognition