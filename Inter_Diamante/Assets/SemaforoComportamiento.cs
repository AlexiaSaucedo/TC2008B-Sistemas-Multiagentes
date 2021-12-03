using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SemaforoComportamiento : MonoBehaviour
{
    public GameObject rojo;
    public GameObject amarillo;
    public GameObject verde;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void setRojo()
    {
        rojo.SetActive(true);
        amarillo.SetActive(false);
        verde.SetActive(false);
    }

    public void setAmarillo()
    {
        rojo.SetActive(false);
        amarillo.SetActive(true);
        verde.SetActive(false);
    }

    public void setVerde()
    {
        rojo.SetActive(false);
        amarillo.SetActive(false);
        verde.SetActive(true);
    }
}
