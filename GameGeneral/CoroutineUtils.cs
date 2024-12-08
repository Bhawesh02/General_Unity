using System;
using System.Collections;
using UnityEngine;

public static class CoroutineUtils
{
    public static IEnumerator Delay(float delayInSeconds, Action finishedCallback)
    {
        yield return new WaitForSeconds(delayInSeconds);
        finishedCallback?.Invoke();
    }
}