Name:           omitag
Version:        1
Release:        1%{?dist}
Summary:        Omitag tag detection component

License:        GPLv2+
URL:            https://github.com/faljse/ftag
Source0:        %{url}/repository/master/archive.tar.gz

BuildRequires:  gcc
BuildRequires:  cmake


%description
ftag is a program... Aruco codes to 3D coordinates based on camera images.


%prep
%global _hardened_build 1
%autosetup

%build
export CFLAGS="%{optflags}"
export LDFLAGS="%{__global_ldflags}"
%make_build


%install
mkdir -p %{buildroot}%{_bindir}
install -p -m 755 %{name} %{buildroot}%{_bindir}/%{name}


%files
%license LICENSE
%{_bindir}/%{name}
%attr(4755, root, root) %{_bindir}/%{name}


%changelog
* Thu Apr 19 2018 Martin Kunz <martin.michael.kunz@gmail.com> - 4-1
- First ftag package
