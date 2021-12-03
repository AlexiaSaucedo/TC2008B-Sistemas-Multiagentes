using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulationParametros : MonoBehaviour
{
    public TextAsset csvParametros;
    public int cantidadCarros;
    public int cantidadSemaforos;
    public int frameRate;
    public List<Vector3> semaforoPositions;
    // Start is called before the first frame update
    void Start()
    {
        ReadCSVFile();
    }

    void ReadCSVFile()
    {
        string[] data = csvParametros.text.Split('\n');
        semaforoPositions = new List<Vector3>();
        for(int i = 0; i < data.Length; i++)
        {
            string[] dataValues = data[i].Split(',');
            if(i == 0)
            {
                cantidadCarros = int.Parse(dataValues[0]);
                cantidadSemaforos = int.Parse(dataValues[1]);
                frameRate = (int) (1.0f / float.Parse(dataValues[3]));
                Application.targetFrameRate = frameRate;
            }
            else
            {
                semaforoPositions.Add(new Vector3(0f,0f,0f));
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
