/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace Jitter2.DataStructures;

/// <summary>
/// A data structure based on an array, without a fixed order. Removing an element at position n
/// results in the last element of the array being moved to position n, with the <see cref="Count"/>
/// decrementing by one.
/// </summary>
/// <typeparam name="T">The type of elements in the SlimBag.</typeparam>
internal class SlimBag<T> : IEnumerable<T>
{
    /// <summary>Lightweight struct enumerator. NOT safe if the bag is
    /// mutated while enumeration is in progress.
    /// </summary>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly SlimBag<T> owner;
        private int index;

        internal Enumerator(SlimBag<T> owner)
        {
            this.owner = owner;
            index = -1;
        }

        public T Current => owner.array[index];
        object? IEnumerator.Current => Current;

        public bool MoveNext()
        {
            // advance and check against the number of valid items, not array.Length
            return ++index < owner.counter;
        }

        // No resources to release
        public void Dispose() { }

        public void Reset() => index = -1;
    }

    private T[] array;
    private int counter;
    private int nullOut;
    private readonly EqualityComparer<T> comparer = EqualityComparer<T>.Default;

    /// <summary>
    /// Initializes a new instance of the <see cref="SlimBag{T}"/> class with a specified initial size.
    /// </summary>
    /// <param name="initialSize">The initial size of the internal array. Defaults to 4 if not specified.</param>
    public SlimBag(int initialSize = 4)
    {
        array = new T[initialSize];
        nullOut = 0;
    }

    /// <summary>
    /// Gets the length of the internal array.
    /// </summary>
    /// <returns>The length of the internal array.</returns>
    public int InternalSize => array.Length;

    /// <summary>
    /// Returns a span representing the valid portion of the internal array.
    /// </summary>
    /// <returns>A <see cref="Span{T}"/> representing the valid portion of the internal array.</returns>
    public Span<T> AsSpan()
    {
        return new Span<T>(array, 0, counter);
    }

    /// <summary>
    /// Adds a range of elements to the <see cref="SlimBag{T}"/>.
    /// </summary>
    /// <param name="list">The collection of elements to add.</param>
    public void AddRange(IEnumerable<T> list)
    {
        foreach (T elem in list) Add(elem);
    }

    /// <summary>
    /// Adds an element to the <see cref="SlimBag{T}"/>.
    /// </summary>
    /// <param name="item">The element to add.</param>
    public void Add(T item)
    {
        if (counter == array.Length)
        {
            Array.Resize(ref array, array.Length * 2);
        }

        array[counter++] = item;
    }

    private Jitter2.Parallelization.ReaderWriterLock rwLock;

    /// <summary>
    /// Adds an element to the <see cref="SlimBag{T}"/>.
    /// </summary>
    /// <param name="item">The element to add.</param>
    public void ConcurrentAdd(T item)
    {
        int lc = Interlocked.Increment(ref counter) - 1;

        again:

        rwLock.EnterReadLock();

        if (lc < array.Length)
        {
            array[lc] = item;
            rwLock.ExitReadLock();
        }
        else
        {
            rwLock.ExitReadLock();

            rwLock.EnterWriteLock();
            if (lc >= array.Length)
            {
                Array.Resize(ref array, array.Length * 2);
            }
            rwLock.ExitWriteLock();

            goto again;
        }
    }

    /// <summary>
    /// Removes the first occurrence of a specific element from the <see cref="SlimBag{T}"/>.
    /// </summary>
    /// <param name="item">The element to remove.</param>
    public void Remove(T item)
    {
        int index = -1;
        for (int i = 0; i < counter; i++)
        {
            if (comparer.Equals(item, array[i]))
            {
                index = i;
                break;
            }
        }

        if (index != -1) RemoveAt(index);
    }

    /// <summary>
    /// Removes the element at the specified index from the <see cref="SlimBag{T}"/>.
    /// </summary>
    /// <param name="index">The zero-based index of the element to remove.</param>
    public void RemoveAt(int index)
    {
        array[index] = array[--counter];
    }

    /// <summary>
    /// Gets or sets the number of elements contained in the <see cref="SlimBag{T}"/>.
    /// </summary>
    public int Count
    {
        get => counter;
        set
        {
            Debug.Assert(value <= counter);
            counter = value;
        }
    }

    /// <summary>
    /// Gets the element at the specified index. Note that index should be smaller
    /// than <see cref="Count"/>, however this is not enforced.
    /// </summary>
    /// <param name="i">The zero-based index of the element to get.</param>
    /// <returns>The element at the specified index.</returns>
    public T this[int i] => array[i];

    /// <summary>
    /// Removes all elements from the <see cref="SlimBag{T}"/>.
    /// </summary>
    public void Clear()
    {
        counter = 0;
    }

    /// <summary>
    /// This should be called after adding entries to the SlimBag
    /// to keep track of the largest index used within the internal array of
    /// this data structure. It will set this item in the array to its default value
    /// to allow for garbage collection.
    /// </summary>
    public void TrackAndNullOutOne()
    {
        nullOut = Math.Max(nullOut, counter);
        if (nullOut <= counter) return;
        array[--nullOut] = default!;
    }

    public Enumerator GetEnumerator() => new Enumerator(this);

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}