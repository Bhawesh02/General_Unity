using System.Collections.Generic;
namespace PathFinding
{
    public class KeyNodeComparer<T> : IComparer<(NodeKey, Node<T>)>
    {
        public int Compare((NodeKey, Node<T>) x, (NodeKey, Node<T>) y)
        {
            NodeKey xKey = x.Item1;
            NodeKey yKey = y.Item1;
            return xKey < yKey ? -1 : xKey > yKey ? 1 : 0;
        }
    }
}
