## Fundamental Algorithms

Binary Search
DFS
BFS
Sliding

Alright — let’s break down those **fundamental algorithms** so that even without coding background, you understand:

* **What they are**
* **When to use them**
* **How they work step-by-step**
* **How you’d think before coding them**

---

## **1. Binary Search**

**What it is:**
A fast way to find an item in a **sorted** list by repeatedly cutting the search range in half.

**When to use:**

* The list is **sorted**.
* You need to **find** something quickly.

**How it works (real-life analogy):**
Think of finding a word in a dictionary:

1. Open the book in the middle.
2. If the word is alphabetically before this page → search left half.
3. If after → search right half.
4. Repeat until you find the word or the section is empty.

**Steps before coding:**

1. Check if input is sorted — if not, sort it (or binary search won’t work).
2. Set `low` = start index, `high` = end index.
3. While `low <= high`:

   * Find middle index.
   * Compare middle value with target.
   * Move search range to left or right half.
4. If found → return index, else return -1.

---

## **2. DFS (Depth-First Search)**

**What it is:**
A way to explore all paths or nodes in a structure (tree, graph) by going **as deep as possible first**, then backtracking.

**When to use:**

* You want to **explore all possibilities** before moving sideways.
* Problems like:

  * Pathfinding in a maze.
  * Checking if something exists in a graph.
  * Traversing a tree.

**How it works (real-life analogy):**
Imagine exploring a cave:

1. Pick a tunnel and go as far as possible.
2. If stuck, go back to the last junction and take another tunnel.

**Steps before coding:**

1. Start at a node.
2. Mark it as visited.
3. Visit one unvisited neighbor → repeat process.
4. Backtrack when no more unvisited neighbors.

**Variants:**

* Recursive DFS (function calls itself).
* Iterative DFS (use a stack).

---

## **3. BFS (Breadth-First Search)**

**What it is:**
A way to explore level-by-level — visiting all neighbors first before going deeper.

**When to use:**

* Find **shortest path** in an unweighted graph.
* Level-order traversal of a tree.
* Checking connectivity.

**How it works (real-life analogy):**
Imagine spreading news in a crowd:

1. You tell all your friends (level 1).
2. They tell their friends (level 2).
3. Repeat until everyone knows.

**Steps before coding:**

1. Start at a node.
2. Put it in a **queue** (FIFO — first in, first out).
3. While queue is not empty:

   * Remove first node.
   * Visit all its unvisited neighbors → put them in queue.
4. Continue until all reachable nodes are visited.

---

## **4. Sliding Window**

**What it is:**
A way to move through an array or list **by keeping track of a portion (window)** and shifting it instead of recalculating from scratch.

**When to use:**

* Problems involving **subarrays, substrings, or ranges**.
* Often for **finding max/min/sum/average** in a range.
* Reduces time complexity from O(n²) to O(n) in many cases.

**How it works (real-life analogy):**
Imagine checking temperature trends over 7 days:

1. Take first 7 days → record result.
2. Slide window by removing day 1 and adding day 8.
3. Repeat.


1. Define window size or condition.
2. Calculate initial result for first window.
3. Move start and end pointers as window slides.
4. Update result based on entering/leaving elements.

