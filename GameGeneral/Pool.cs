using System.Collections.Generic;
using UnityEngine;

public class Pool<T> where T : Component
{
    private Queue<T> m_elements;
    private T m_prefab;
    private bool m_dynamicSize;
    private Transform m_defaultParent;

    public Pool(bool dynamicSize, T prefab, int initialSize, Transform defaultParent)
    {
        m_elements = new Queue<T>();

        m_dynamicSize = dynamicSize;
        m_prefab = prefab;
        m_defaultParent = defaultParent;

        for (int j = 0; j < initialSize; ++j)
        {
            T obj = Object.Instantiate(m_prefab, m_defaultParent, false);
            obj.transform.name = m_prefab.transform.name;
            obj.gameObject.SetActive(false);
            m_elements.Enqueue(obj);
        }
    }

    public void Clear()
    {
        while (m_elements.Count > 0)
        {
            T element = m_elements.Dequeue();
            Object.Destroy(element);
        }
        m_elements.Clear();
    }

    public T GetElement()
    {
        return GetElement(m_defaultParent);
    }

    public T GetElement(Transform newParent, bool worldPositionStays = true)
    {
        T element = null;
        newParent ??= m_defaultParent;

        if (m_elements.Count > 0)
        {
            element = m_elements.Dequeue() as T;
        }
        else
        {
            if (m_dynamicSize)
            {
                element = Object.Instantiate(m_prefab,m_defaultParent, worldPositionStays) as T;
                element.name = m_prefab.name;
            }
        }
        if (element)
        {
            element.transform.SetParent(newParent, worldPositionStays);
            element.gameObject.SetActive(true);
        }
        return element;
    }

    public void ReturnElement(T component)
    {
        if (m_elements.Contains(component))
        {
            return;
        }
        component.transform.SetParent(m_defaultParent, true);
        m_elements.Enqueue(component);
        component.gameObject.SetActive(false);
    }
}