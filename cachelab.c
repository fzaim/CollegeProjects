/* Farjad Zaim, fzaim */
#include "cachelab.h"
#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include<stdlib.h>

int main(int argc, char **argv)
{
	int opt, s, E, b;
	char *t;
	while (-1 != (opt = getopt(argc, argv, "s:E:b:t:")))
	{
		switch(opt)
		{
			case 's': 
				s=atoi(optarg);
				break;
			case('E'):
				E=atoi(optarg);
				break;
			case('b'):
				b = atoi(optarg);
				break;
			case('t'):
				t = optarg;
				break;	
		}
	}
	FILE *file =fopen(t, "r");
	int hits = 0; int isHit;
	int misses = 0; int isMiss;
	int evictions = 0; int isMissEvict;
     	char type;
	int S = 1<<s;
	if (file != NULL)
	{	
		char line[100];
		unsigned address0;
		unsigned currentTag;
		unsigned setIndex;
		int valid[S][E];
		unsigned tags[S][E];
		int age[S][E];
		int i; int j;
		for (i =0; i<S; i++)
			for(j = 0; j<E; j++)
			{
				valid[i][j] = 0;
				tags[i][j] =0;
				age[i][j] =0;
			}
		fscanf(file, " %c %x", &type, &address0);
		currentTag = address0>>(s+b); 
		setIndex = address0<<(64-s-b);
		setIndex = setIndex>>(64-s);
		valid[setIndex][0] = 1;
		tags[setIndex][0] = currentTag;
		misses++;		 	
		
		while (fgets(line, 100, file) != NULL)
		{	
			int extraHit = 0;
			isHit =0; isMiss =0; isMissEvict =0;
			unsigned address;	 
			fscanf(file, " %c %x", &type, &address);
			if (type == 'M')
				extraHit = 1;
			if (type == 'M' || type =='S' || type == 'L')
			{
			currentTag = address>>(s+b); 
			setIndex = address<<(64-s-b);
			setIndex = setIndex>>(64-s);
			int hasInvalid = 0;
			int x;
			for(x=0; x<E; x++)
				age[setIndex][x] = age[setIndex][x] +1;
			for(x =0; x<E; x++)
			{
				if (valid[setIndex][x] ==1)
				{
					if (tags[setIndex][x] ==currentTag)
					{
						age[setIndex][x] = 1;
						hits++; 
						isHit = 1;
						break;
					}
				}
			}
			if (isHit ==0)
				for(x =0; x<E; x++)
					if(valid[setIndex][x] ==0)
					{
						age[setIndex][x] = 1;
						misses++;
						valid[setIndex][x] =1;
						tags[setIndex][x] = currentTag;
						hasInvalid = 1;
						isMiss = 1;
						break;
					}
			if (isHit ==0 && hasInvalid ==0)
			{
				int maxAge = 0; int maxX =0;
				for (x = 0; x<E; x++)
				{
					if(age[setIndex][x] >=maxAge)
					{
						maxAge = age[setIndex][x];	
						maxX = x;

						break;
					}
				}
				age[setIndex][maxX] = 1;
				tags[setIndex][maxX] = currentTag; 
				misses++; evictions++; 
				isMissEvict = 1;
			}
			}
			if (extraHit ==1)
			{
				hits++;
			}
			
		}
		fclose(file);
	}
	hits = hits - isHit;
	misses = misses -isMiss -isMissEvict;
	evictions = evictions - isMissEvict;
	if (type == 'M')
	 	hits = hits -1;
	printSummary(hits, misses, evictions);
	return 0;
}
