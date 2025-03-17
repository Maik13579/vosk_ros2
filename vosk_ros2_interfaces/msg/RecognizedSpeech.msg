Result[] result             # List of recognized words with timestamps/confidence
string text                 # Complete recognized text from this chunk
Speaker[] spker             # List of speakers ordered my cosine distance
float32[] spk_embedding     # Speaker embedding (if speaker recognition is enabled)
int32 spk_frames            # Number of speaker frames
Alternative[] alternatives  # Alternate transcriptions for this speech segment
float32 elapsed_time        # Time elapsed (in seconds) for this recognition