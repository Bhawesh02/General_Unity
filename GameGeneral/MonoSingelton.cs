using System;
using UnityEngine;

public abstract class MonoSingleton<T> : MonoBehaviour where T : MonoSingleton<T>
{

    public static T Instance { get; private set; }

    protected virtual void Awake()
    {
        if (Instance == null)
        {
            Instance = (T)this;
        }
        else
        {
            Destroy(gameObject);
        }
        Init();
    }
    protected abstract void Init();
}