#!/usr/bin/env python3
import numpy as np
import json
import os

class SpeakerDatabase:
    def __init__(self, path: str):
        """
        Initializes the speaker database.

        :param path: File path where speaker data is stored.
        """
        self.path = path
        self.speakers = {}  # Dictionary mapping speaker ID (string) to their embedding (numpy array)
        if self.path is None or self.path.strip() == "":
            self.path = None
        else:
            self.load()

    def add_speaker(self, speaker_id: str, embedding: list):
        """
        Adds a speaker to the database and saves the updated database to file.

        :param speaker_id: Unique identifier for the speaker.
        :param embedding: List of floats representing the speaker's embedding.
        """
        self.speakers[speaker_id] = np.array(embedding, dtype=np.float32)
        if self.path:
            self.save()

    def compute_cosine_distance(self, vec1: np.ndarray, vec2: np.ndarray) -> float:
        """
        Computes the cosine distance between two vectors.

        Cosine distance is defined as:
           1 - (dot(vec1, vec2) / (||vec1|| * ||vec2||))

        :param vec1: First vector (numpy array).
        :param vec2: Second vector (numpy array).
        :return: Cosine distance as a float. Returns 1.0 if either vector is zero.
        """
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)
        if norm1 == 0 or norm2 == 0:
            return 1.0
        cosine_similarity = np.dot(vec1, vec2) / (norm1 * norm2)
        return 1.0 - cosine_similarity

    def get_sorted_speakers(self, current_embedding: list):
        """
        Computes the cosine distance from the current embedding to each speaker
        in the database. Returns a sorted list of tuples (speaker_id, cosine_distance)
        in ascending order (most similar first).

        :param current_embedding: List of floats representing the current speaker's embedding.
        :return: Sorted list of tuples, e.g., [("speaker1", 0.123), ("speaker2", 0.234), ...]
        """
        current_vec = np.array(current_embedding, dtype=np.float32)
        results = []
        for speaker_id, emb in self.speakers.items():
            distance = self.compute_cosine_distance(current_vec, emb)
            results.append((speaker_id, distance))
        results.sort(key=lambda x: x[1])
        return results

    def save(self):
        """
        Saves the current speaker database to the file specified by self.path.
        Speaker embeddings (numpy arrays) are converted to lists.
        """
        data = {speaker_id: emb.tolist() for speaker_id, emb in self.speakers.items()}
        with open(self.path, "w") as f:
            json.dump(data, f, indent=2)

    def load(self):
        """
        Loads the speaker database from the file specified by self.path.
        If the file does not exist, the database starts empty.
        """
        if os.path.exists(self.path):
            with open(self.path, "r") as f:
                data = json.load(f)
            self.speakers = {speaker_id: np.array(emb, dtype=np.float32) for speaker_id, emb in data.items()}
        else:
            self.speakers = {}