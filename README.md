# Flight Crash Analysis and Prevention using AIOPs
Simple stochastic aircraft metrics model with different failure vectors. How sick does that sound?

**Accenture Bootcamp July 2019 resource.** This project is the backing application built using [Python 3.7](https://www.python.org/downloads/release/python-370/) that creates mock aircraft data, sends it to an API hosted using [JSON Server](https://github.com/typicode/json-server), and listens for remediation messages to alter the data being generated.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine. Currently, these instructions only support Win10 systems.

### Prerequisites

1. Python 3 (https://www.python.org/downloads/release/python-370/, recommend 3.5+)
2. Git (https://git-scm.com/, *Optional*)

### Installing

#### Python

First, install the prerequisites through binaries available on the websites linked above. These set up Python 3.7 and Git on the local machine if they aren't already.

Next, we'll install our Python prerequisites through command line (recommend Windows PowerShell) - our libraries that aren't part of the initial Python package that are needed to connect with everything. The `numpy` library is used to generate the data efficiently. The `pyyaml` library is needed to read the settings, which we'll set up in just a moment. The `requests` library is needed to correctly make REST calls to the JSON server API.

```
pip install numpy
pip install pyyaml
pip install requests
```

Make a new directory somewhere for the project and cd into it in command line. Clone this repository into this directory (if using Git, otherwise, just download this repository and extract).

```
cd \Documents\Accenture\ && mkdir flight-crash-aiops\ && cd flight-crash-aiops\
git clone https://github.com/charlingli/flight-crash-aiops.git
```

#### To be completed

### Setting Up

## Deployment

## Testing

## Contributing

Send me a message or just chat to me :)

### Limitations/Future work


## Troubleshooting

## Authors

- **Charling Li** - [charlingli](https://github.com/charlingli)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
