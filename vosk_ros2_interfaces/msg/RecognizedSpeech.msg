Result[] results            # List of recognized words with timestamps/confidence
string text                 # Complete recognized text from this chunk
string spk                  # Speaker ID (if speaker recognition is enabled)
float32[] spk_embedding     # Speaker embedding (if speaker recognition is enabled)
int32 spk_frames            # Number of speaker frames
Alternative[] alternatives  # Alternate transcriptions for this speech segment
float32 elapsed_time        # Time elapsed (in seconds) for this recognition