# Large File Storage Note

Some original video files in this archived project exceeded GitHub's 100 MB file limit.

To preserve them in the repository, they are stored as split parts in the same folders as the original videos.

Reassembly examples:

```bash
cat "Task 2 Video.mp4.part-"* > "Task 2 Video.mp4"
cat "Task 3 Video.mp4.part-"* > "Task 3 Video.mp4"
cat "Bonus Task Video.mp4.part-"* > "Bonus Task Video.mp4"
cat "Task 4 Video.mp4.part-"* > "Task 4 Video.mp4"
```

Run the command from the directory that contains the split parts for the corresponding video.
