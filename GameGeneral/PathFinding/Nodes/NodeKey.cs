using System;
using UnityEngine;

namespace PathFinding
{
    public struct NodeKey
    {
        public float primaryKey;
        public float secondaryKey;

        public NodeKey(float primaryKey, float secondaryKey)
        {
            this.primaryKey = primaryKey;
            this.secondaryKey = secondaryKey;
        }

        public static bool operator <(NodeKey a, NodeKey b) =>
            a.primaryKey < b.primaryKey || Mathf.Approximately(a.primaryKey, b.primaryKey)
            && a.secondaryKey < b.secondaryKey;

        public static bool operator >(NodeKey a, NodeKey b) =>
            a.primaryKey > b.primaryKey || Mathf.Approximately(a.primaryKey, b.primaryKey)
            && a.secondaryKey > b.secondaryKey;

        public static bool operator ==(NodeKey a, NodeKey b) =>
            Mathf.Approximately(a.primaryKey, b.primaryKey) && Mathf.Approximately(a.secondaryKey, b.secondaryKey);

        public static bool operator !=(NodeKey a, NodeKey b) =>
            !(a == b);

        public override bool Equals(object obj) =>
            obj is NodeKey key
            && this == key;

        public override int GetHashCode() => HashCode.Combine(primaryKey, secondaryKey);

        public override string ToString() => $"{primaryKey} , {secondaryKey}";
    }
}