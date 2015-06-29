/*  Yi Huo (yhuo)
 *  Farjad Zaim (fzaim)
 */

#include <stdio.h>
#include "csapp.h"
#include <pthread.h>

#define MAX_CACHE_SIZE 1049000
#define MAX_OBJECT_SIZE 102400
#define MAX_NUM_WEBPAGE 10
#define HOST_LOC         7

static const char *user_agent = "User-Agent: Mozilla/5.0 (X11; Linux x86_64; rv:10.0.3) Gecko/20120305 Firefox/10.0.3\r\n";
static const char *accepts = "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8\r\n";
static const char *accept_encoding = "Accept-Encoding: gzip, deflate\r\n";
static const char *connection = "Connection: close\r\n";
static const char *proxy_connection = "Proxy-Connection: close\r\n";

/* Function Prototypes */
int formatRequest(int client_connfd, int *port, char *actualHost, char *request, char *aUri);
void echo (int connfd);
void *handle_client(void *connfd);
int open_clientfd_r(char *hostname, int port);
int findpage(char *host, char *uri);
void writepage(char *host, char *uri, char *page, int *size);
void find_lru_page(int *slot);
int readpage(char *host, char *uri, char **page);

/* Webpage struct used for cache */
typedef struct page {
  char *host;
  char *uri;
  char *web_object;
  int pagesize;
} webpage;

/* Global Variables */
webpage *cache[MAX_NUM_WEBPAGE];    /* Cache */
int lru[MAX_NUM_WEBPAGE];           /* Keeps track of lru replacement */
int readcnt;                        /* Number of readers */
sem_t r, w, l;                      /* Reader, writer and lru locks */

int main(int argc, char **argv)
{
	int listenfd, *client_connfd, listeningPort, clientlen, i;
	struct sockaddr_in clientaddr;
	sigset_t mask;
	pthread_t tid;

  // Prevent SIGPIPE from interrupting our proxy
	Sigemptyset(&mask);
	Sigaddset(&mask, SIGPIPE);
	Sigprocmask(SIG_BLOCK, &mask, NULL); 

  // Initialize global variables
  readcnt = 0;
  Sem_init(&r, 0, 1);
  Sem_init(&w, 0, 1);
  Sem_init(&l, 0, 1);

  // Initialize cache
  P(&w);
  for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
    cache[i] = calloc(1, sizeof(webpage));
    cache[i]->host = calloc(1, sizeof(char)*MAXLINE);
    cache[i]->uri = calloc(1, sizeof(char)*MAXLINE);
    cache[i]->web_object = calloc(1, sizeof(char)*MAX_OBJECT_SIZE);
  }
  V(&w);
  
  // Initialize LRU
  P(&l);
  printf("[");
  for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
    lru[i] = MAX_NUM_WEBPAGE - i;
    printf("%d, ", lru[i]);
  }
  printf("]\n");
  V(&l);

  // Get listening port number
	listeningPort = atoi(argv[1]);
	printf("listeningPort = %d\n", listeningPort);
  printf("%s%s%s", user_agent, accepts, accept_encoding);

	if (argc!=2){
		fprintf(stderr, "usageL %s <port>\n", argv[0]);
		exit(0);
	}

  // Open listening port and start listening
	listenfd = open_listenfd(listeningPort);
	printf("listenfd = %d\n", listenfd);

  // Accept connections and fork a thread to handle the request
	while(1){
		clientlen = sizeof(clientaddr);
		client_connfd = calloc(1, sizeof(int));
		*client_connfd = Accept(listenfd, (SA *)&clientaddr, &clientlen);

    // Handle the request in a thread
		Pthread_create(&tid, NULL, handle_client, client_connfd);
	}

	exit(0);
}


/*  handle_client
 *
 *  Process a request from the client
 */
void *handle_client(void *connfd) {
		char *buf, *host, *newRequest, *temp, *uri, *page;
    char *readpagebuf; 
		int serverPort, serverfd, read;
		rio_t rio;
		size_t n;
    int *bytes_read;

		int client_connfd = *((int *)connfd); /* Get client file descriptor */
   
    Pthread_detach(pthread_self()); 
		
    free(connfd);       /* Free memory */

    // Initialize local variables, use calloc to prevent sharing
    page = calloc(1, sizeof(char)*MAX_OBJECT_SIZE);
    uri = calloc(1, sizeof(char)*MAXLINE);
		buf = calloc(1, sizeof(char)*MAXLINE);
		host = calloc(1, sizeof(char)*MAXLINE);
		newRequest = calloc(1, sizeof(char)*MAXLINE);
		temp = calloc(1, sizeof(char)*MAXLINE);
		
		printf("client_connfd = %d\n", client_connfd);

		serverPort = 80;		                /* Default server port number */
		
    printf("Calling formatRequest\n");
		printf("Host address = %s\n", host);

    // Process only GET requests, ignore all other requests
		if(formatRequest(client_connfd, &serverPort, host, newRequest, uri) 
        == 1)		
		{	
			printf("Returned from formatRequest\n");
			printf("Host = %s\n", host);
			printf("Server Port = %d\n", serverPort);

      // Retrieve page from server if not found in cache
      if(findpage(host, uri) == 0) {

        bytes_read = calloc(1, sizeof(int));
        *bytes_read = 0;          /* Size of page retrieved from server */
			  // Open a connection to the server
        serverfd = open_clientfd_r(host, serverPort); 
			  printf("Serverfd = %d\n", serverfd);
			  rio_readinitb(&rio, serverfd); /* Get server response */

			  printf("Start to receive data from server\n");
			  printf("New Request = %s\n", newRequest);
		
        // Send request to server
			  rio_writen(serverfd, newRequest, strlen(newRequest));
			
		    printf("Page pointer before going to server = %p\n", page);	
			  printf("Finished sending request to server\n");

        // Get response from server and forward response to client
			  while((n = rio_readnb(&rio, buf, MAXLINE))  != 0) {
				  rio_writen(client_connfd, buf, n); /* Forward response to client */
          
          // Copy response from server if it doesn't exceed max size
          if((*bytes_read)+n <= MAX_OBJECT_SIZE)
            memcpy(page+(*bytes_read), buf, n); /* Copy response into buf */
          *bytes_read += n;
          printf("bytes_read = %d\n", *bytes_read);
			  }
			  Close(serverfd);  /* Close connection to server */

        printf("Page pointer after going to server = %p\n", page);

        // Cache the page if it does not exceed max size
        if((*bytes_read) <= MAX_OBJECT_SIZE)
          writepage(host, uri, page, bytes_read);
        else
          printf("Page too big to be cached\n");
      }
      // Retrieve page from cache and forward it to the client
      else {
        read = readpage(host, uri, &readpagebuf); /* Get page from cache */
        printf("Page size from cache = %d\n", read);
        rio_writen(client_connfd, readpagebuf, read); /* Send page to client */
      }
		}

		Close(client_connfd);       /* Close connection to client */
    // Free all memory resources
    free(page);
    free(uri);
		free(buf);
		free(host);
		free(newRequest);
		free(temp);
		return NULL;
}

/*  readpage
 *
 *  Retrieve a page from the cache
 */
int readpage(char *host, char *uri, char **page) {
  int i, page_size = -1, index;

  // Increase number of cache readers
  P(&r);
    readcnt++;
    if(readcnt == 1)
      P(&w);              /* Block writers */
  V(&r);

  // Find the webpage in the cache
  for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
    if((strcmp(cache[i]->host, host) == 0) &&
       (strcmp(cache[i]->uri, uri) == 0)) 
    {
       index = i;                      /* Location of webpage in the cache */   
       *page = cache[i]->web_object;   /* Pointer to the webpage */
       page_size = cache[i]->pagesize; /* Size of the webpage */
    }
  }

  // Update LRU
  P(&l);
    printf("LRU in readpage: [");
    for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
      if(lru[i] < lru[index])
        lru[i]++;
      printf("%d, ", lru[i]);
    }
    lru[index] = 1;
    printf("]\n");
  V(&l);  

  // Decrement number of cache readers
  P(&r);
    readcnt--;
    if(readcnt == 0)  /* Unblock writers */
      V(&w);  
  V(&r);

  return page_size;
}

/*  writepage
 *
 * Store a page in the cache
 */
void writepage(char *host, char *uri, char *page, int *size) {
  int *freeslot, i;

  freeslot = calloc(1, sizeof(int)); /* Position of a free slot in cache */

  // Lock out all other readers and writers
  P(&w);
  P(&l);
    find_lru_page(freeslot);     /* Find a free slot in cache */

    // Replacement policy is bugged
    if(*freeslot == -1) {
      printf("LRU is messed up in writepage\n");
      exit(1);
    }

    printf("Starting to free the lru webpage in cache\n");
    // Free the previous webpage
    free(cache[*freeslot]->host);
    free(cache[*freeslot]->uri);
    free(cache[*freeslot]->web_object);
    
    printf("Allocate space for new webpage in writepage\n");
    // Allocate space for the new webpage
    cache[*freeslot]->host = calloc(1, sizeof(char)*MAXLINE);
    cache[*freeslot]->uri = calloc(1, sizeof(char)*MAXLINE);
    cache[*freeslot]->web_object = calloc(1, sizeof(char)*MAX_OBJECT_SIZE);

    // Copy over the webpage
    strcpy(cache[*freeslot]->host, host);
    strcpy(cache[*freeslot]->uri, uri);
    memcpy(cache[*freeslot]->web_object, page, *size);
    printf("Size of page written is %d\n", *size);
    cache[*freeslot]->pagesize = *size;
    
    // Update LRU
    lru[*freeslot] = 0;
    for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
      lru[i]++;
    }

    // Print out the size of each object in the cache
    printf("[");
    for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
      printf("%d, ", cache[i]->pagesize);
    }
    printf("]\n");

    // Free memory resources
    printf("Freeing freeslot in writepage\n");
    free(freeslot);
    printf("Freeing size in writepage\n");
    free(size);
  V(&l);
  V(&w);        /* Release the writers lock */
}

/*  find_lru_page
 *
 *  Find a free slot in the cache
 */
void find_lru_page(int *slot) {
  int i;
  *slot = -1;               

  printf("[");
  for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
    if(lru[i] == MAX_NUM_WEBPAGE) {
      *slot = i;        /* Found a free slot */
    }
    printf("%d, ", lru[i]);
  }
  printf("]\n");
}

/*  findpage
 *
 *  Check to see if a webpage is in the cache
 */
int findpage(char *host, char *uri) {
  int hit = 0, i;

  printf("Incrementing readers in findpage\n");
  // Increment number of readers
  P(&r);
    readcnt++;
    if(readcnt == 1)
      P(&w);      /* Block writers */
  V(&r);

  printf("Searching cache for page in findpage\n");
  // Search cache for the webpage
  for(i = 0; i < MAX_NUM_WEBPAGE; i++) {
    if((strcmp(cache[i]->host, host) == 0) &&
       (strcmp(cache[i]->uri, uri) == 0)) 
    {
      printf("cache host = %s\n", cache[i]->host);
      printf("host = %s\n", host);
      printf("cache uri = %s\n", cache[i]->uri);
      printf("uri = %s\n", uri);
      hit = 1;      /* Found the page */
      break;
    }
  }

  printf("Decrementing readers in findpage\n");
  // Decrement the number of readers
  P(&r);
    readcnt--;
    if(readcnt == 0)  /* Unblock writers */
      V(&w);
  V(&r);

  return hit;       /* Return the result of this search */
}

/*  open_clientfd_r
 *
 *  Thread safe. Open a connection to the client
 */
int open_clientfd_r(char *hostname, int port) 
{
  int clientfd;
	char portnum[16];
	int rv;
	struct addrinfo *serverinfo, *d;

	sprintf(portnum, "%d", port);         /* Get client's port number */
	printf("%s\n", portnum);
	
  // Get info on the client
	if((rv = getaddrinfo(hostname, portnum, NULL, &serverinfo)) != 0) {
		printf("getaddrinfo failed\n");
		exit(1);
	}

	// Find a client we can connect to
	for(d = serverinfo; d != NULL; d = d->ai_next) {
    if ((clientfd = socket(d->ai_family, d->ai_socktype,
            d->ai_protocol)) == -1) {
        printf("Failed to get socket in open_clientfd_r\n");
        continue;
    }

    if (connect(clientfd, d->ai_addr, d->ai_addrlen) == -1) {
        close(clientfd);
        printf("Failed to connect to client in open_clientfd_r\n");
        continue;
    }

    break; // Connected successfully
	}

  // Failed to find a connection
  if(d == NULL) {
    fprintf(stderr, "failed to connect\n");
    exit(2);
  }

  freeaddrinfo(serverinfo); // Free memory resources

  return clientfd;
}

/*  formatRequest
 *
 *  Change the request into an appropriate format
 *  for sending to the server
 */
int formatRequest(int client_connfd, int *port, char *actualHost, 
    char *request, char *aUri){
	char *temp, *buf, *host, *method, *uri, *version, *newUri, *extra_headers;
	rio_t rio;
	char *requestedPort;
	int numScanned, currIndex, n;

  // Calloc local variables to avoid sharing
	buf = calloc(1, sizeof(char)*MAXLINE);
	host = calloc(1, sizeof(char)*MAXLINE);
	method = calloc(1, sizeof(char)*MAXLINE);
	uri = calloc(1, sizeof(char)*MAXLINE);
	version = calloc(1, sizeof(char)*MAXLINE);
	newUri = calloc(1, sizeof(char)*MAXLINE);
	requestedPort = calloc(1, sizeof(char)*MAXLINE);
	extra_headers = calloc(1, sizeof(char)*MAXLINE);
	temp = calloc(1, sizeof(char)*MAXLINE);

	printf("Calling readinitb\n");
	rio_readinitb(&rio, client_connfd); /* Get response from client */
	printf("Calling readlineb\n");
  // Get request from client
	while((n = rio_readlineb(&rio, temp, MAXLINE)) > 2) {
		if((strstr(temp, "User-Agent") == NULL) && 
			(strstr(temp, "Accept") == NULL) &&
			(strstr(temp, "Accept-Encoding") == NULL) && 
			(strstr(temp, "Proxy-Connection") == NULL) &&
			(strstr(temp, "Connection") == NULL) &&
			(strstr(temp, "HTTP") == NULL) &&
			(strstr(temp, "Host") == NULL)) {
			strcat(extra_headers, temp);
		}
    
    // Line with the type of request
		if(strstr(temp, "HTTP") != NULL)
			strcat(buf, temp);

		printf("n = %d, %s\n", n, buf);
	}
	printf("Scanning\n");
  // Find out the uri and type of request
	numScanned = sscanf(buf, "%s %s", method, uri);
	printf("Number of items scanned = %d\n", numScanned);
	printf("Scanning complete\n");
	printf("Buffer: %s \n", buf);
	printf("Method: %s\n", method);
	printf("URI: %s\n", uri);
  // Process GET requests, ignore all others
	if (strcasecmp(method, "GET")){
		//Error Message
		printf("Can Only Process GET requests\n");
		return -1;
	}
	else
		printf("GET request\n");

	currIndex = HOST_LOC;     /* Starting place for host in buf */

	// Get Host string
	printf("Get Host String\n");
	strcat(host, "Host: ");
	while(uri[currIndex] != '/' && uri[currIndex] != ':' && uri[currIndex] != '\0'){
		strncat(host, &uri[currIndex], 1);
		strncat(actualHost, &uri[currIndex], 1);
		currIndex++;
	}

	printf("Host = %s\n", host);
	
	// Get port number if there is one
	printf("Get Port Number\n");
	if(uri[currIndex] == ':') {
		currIndex++;
		while(uri[currIndex] != '\0' && uri[currIndex] != '/') {
			strncat(requestedPort, &uri[currIndex], 1);
			currIndex++;
		}
		*port = atoi(requestedPort);
	}

	// Get URI
	printf("Getting URI\n");
	while(uri[currIndex] != '\0') {
		strncat(newUri, &uri[currIndex], 1);
		currIndex++;
	}
	printf("URI = %s\n", newUri);

  // Put all the necessary fields together into a new request
  strcpy(aUri, newUri);
	strcat(host, "\r\n");
	strcat(version, "HTTP/1.0\r\n");
	strcat(request, method);
	strcat(request, " ");
	strcat(request, newUri);
	strcat(request, " ");
	strcat(request, version);
	strcat(request, host);
	strcat(request, user_agent);
	strcat(request, accepts);
	strcat(request, accept_encoding);
	strcat(request, connection);
	strcat(request, proxy_connection);
	strcat(request, extra_headers);
	strcat(request, "\r\n");
	printf("%s\n", request);

  // Free memory resources
	free(buf);
	free(host);
	free(method);
	free(uri);
	free(version);
	free(newUri);
	free(requestedPort);
	free(extra_headers);
	return 1;
}
